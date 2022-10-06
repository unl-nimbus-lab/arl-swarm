/* This implements the preparation of the complete state space for the HRL-LQI controller.
Here's how the states must be arranged to be fed into the hrl_lqr to compute the transformed states.
Assume the groups are [2 1]. Group 1 has 2 agents, group 2 has 1 agent.
This grouping corresponds to the notation used in the original hrl-lqi code by ARL.

   [       ------------
   x1                  |
   y1                  |
   x2                  |
   y2                  |
\dot(x1)               |
\dot(y1)          group 1
\dot(x2)               |
\dot(y2)               |
integral x1            |
integral y1            |
integral x2            |
integral y2            |
		  -------------
   x3     -------------
   y3                  |
\dot(x3)               |
\dot(y3)            group 2
integral x3            |
integral y3            |
   ]      -------------

I am directly accessing global variables where I should not be doing. That's becuase we do not know how some of those would be initiated.
*/
#include <stdlib.h>
#include <array>
#include <ros/ros.h>
#include <thread>
#include <string>
#include <mutex>
#include "std_msgs/String.h"
#include <math.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <custom_libs/geo_converter.hpp>
#include <custom_libs/json_reader.hpp>
#include <custom_libs/id_to_idx.hpp>
#include <swarm_msgs/GlobalStates.h>
#include <swarm_msgs/gps.h>
#include <swarm_msgs/signal.h>
#include <mavros_msgs/DebugValue.h>
#include <mavros_msgs/SwarmGPS.h>
#include <mavros_msgs/SwarmOdom.h>
#include <chrono>
#include <time.h>
#include <tuple>

#define ROS_NODE_NAME "pre_hrl"
#define CONTROLLER_RATE 50	 // 8    // Rate in Hz
#define INTRGL_RESET_TRGGR 1 // The RESET button in the GUI send 1

#define INTGRL_MTPL 0.01 // A scalar multiplier to ewduce the magnitude of the integral state

// float state[12] = {0.0} ; // stores all the states
// float intgrl_state[4] = {0.0} ;
std::mutex state_mutex; // mutex for state variable

geo_converter gc; // object to call gps coordinate converter functions
id_to_idx id2idx;

float *state = NULL;		// vector to hold all the state values
float *intgrl_state = NULL; // vector to hold the integral states
float *target_lat = NULL;	// vector to hold the targets
float *target_lon = NULL;
int *groups = NULL; // vector to hold the agent clusters in the swarm
int *id_map = NULL;

int hrl_id;	  // ID of the agent in the swarm. This is different from the dron ID
int N;		  // total number of agents
int n_groups; // number of groups

swarm_msgs::GlobalStates hrl_msg;
std::mutex hrl_msg_mutex;

bool use_gps = true;
std::mutex signal_mutex;

/* QP stuff */
float base_lat;
float base_lon;

// float target_lat ;
// float target_lon ;

float own_target_x;
float own_target_y;

// global variables for state estimator
float *updated_ag_x = NULL;
float *updated_ag_y = NULL;
float *updated_ag_vx = NULL;
float *updated_ag_vy = NULL;
float *updated_ag_tgt_x = NULL;
float *updated_ag_tgt_y = NULL;

float ag_x, ag_y, ag_tgt_x, ag_tgt_y, ag_vx, ag_vy;
int ag_id, ag_itgrl_idx, ag_state_idx;
bool gps_update = false;
float t_prev = 0;

std::string topic_to_hrl = "/swarm/global_states/";
std::string topic_to_dist_to_target = "/swarm/dist_to_target/";
// std::string topic_to_debug_vector = "/mavros/debug_value/debug_vector" ;
std::string topic_to_debug_vector = "/mavros/debug_value/send";

void gpsCallback(const mavros_msgs::SwarmGPS::ConstPtr &new_msg)
{

	// This function reads the callback messgaes, re-arranges them to the specific order of the formation
	int incoming_ag_id = new_msg->system_id;

	// Find the hrl_id of the agent with a mapping FCU id
	auto idx = std::find(id_map, id_map + N, incoming_ag_id);
	// If element was found
	if (idx != id_map + N)
	{
		ag_id = idx - id_map + 1; // agent ids are from 1 to N
	}

	try
	{
		// Get the corresponding index in the transformed state space given an agent id
		ag_state_idx = id2idx.get_state_idx(ag_id, groups);
		ag_itgrl_idx = id2idx.get_integral_idx(ag_id, groups);
	}
	catch (const char *e)
	{
		// Exit the program
		std::cout << " First, Check your id  mapping" << std::endl;
		std::cout << "exception: " << e << std::endl;
		exit(EXIT_FAILURE);
	}

	// We need to check to which group does the agnet belong to
	int grp_nbr = id2idx.get_group_nbr(ag_id, groups);

	// Calculate state values from the new msg
	float ag_x = gc.lon_to_meters_signed(base_lat, base_lon, base_lat, new_msg->longitude); // x state with respect to the base (0,0)
	float ag_y = gc.lat_to_meters_signed(base_lat, base_lon, new_msg->latitude, base_lon);	// y state with respect to the base (0,0)

	// added this from below if statement
	float ag_tgt_x = gc.lon_to_meters_signed(base_lat, base_lon, base_lat, target_lon[grp_nbr - 1]); // x distance to taregt with respect to the base (0,0)
	float ag_tgt_y = gc.lat_to_meters_signed(base_lat, base_lon, target_lat[grp_nbr - 1], base_lon); // y distance to taregt with respect to the base (0,0)

	updated_ag_x[ag_id - 1] = ag_x;
	updated_ag_y[ag_id - 1] = ag_y;
	updated_ag_tgt_x[ag_id - 1] = ag_tgt_x;
	updated_ag_tgt_y[ag_id - 1] = ag_tgt_y;

	// // Update the integral state if the id is its id
	// if (std::abs(ag_id - hrl_id) < 0.000000001)
	// {

	// 	float ag_tgt_x = gc.lon_to_meters_signed(base_lat, base_lon, base_lat, target_lon[grp_nbr - 1]); // x distance to taregt with respect to the base (0,0)
	// 	float ag_tgt_y = gc.lat_to_meters_signed(base_lat, base_lon, target_lat[grp_nbr - 1], base_lon); // y distance to taregt with respect to the base (0,0)

	// 	intgrl_state[0] += (ag_x - ag_tgt_x) * INTGRL_MTPL; // multiply by a scaler to reduce the magnitiude
	// 	intgrl_state[1] += (ag_y - ag_tgt_y) * INTGRL_MTPL; // multiply by a scaller to reduce the magnitiude

	// 	// And save the current locations as well
	// 	own_target_x = ag_tgt_x - ag_x;
	// 	own_target_y = ag_tgt_y - ag_y;
	// }

	// // Re-arrange stuff
	// const std::lock_guard<std::mutex> lock(state_mutex);
	// // Copy the value from the fcu to the state variable
	// state[ag_state_idx] = ag_x;
	// state[ag_state_idx + 1] = ag_y;

	// // Copy the integral states to the state vector
	// if (std::abs(ag_id - hrl_id) < 0.000000001)
	// {
	// 	state[ag_itgrl_idx] = intgrl_state[0];
	// 	state[ag_itgrl_idx + 1] = intgrl_state[1];
	// }

	gps_update = true;
}

void velocityCallback(const mavros_msgs::SwarmOdom::ConstPtr &new_msg)
{

	// This function reads the callback messgaes, re-arranges them to the specific order of th>
	//        int ag_id = new_msg->system_id ;
	int ag_vel_idx;
	int incoming_ag_id = new_msg->system_id;

	// Find the hrl_id of the agent with a mapping FCU id
	auto idx = std::find(id_map, id_map + N, incoming_ag_id);
	// If element was found
	if (idx != id_map + N)
	{
		ag_id = idx - id_map + 1; // agent ids are from 1 to N
	}

	try
	{
		// Get the corresponding index in the transformed state space given an agent id
		ag_vel_idx = id2idx.get_velocity_idx(ag_id, groups);
		//              ROS_INFO("@@@@@@@@ ag_id :%d     vel index : %d", ag_id, ag_vel_idx);
	}
	catch (const char *e)
	{
		// Exit the program
		std::cout << " First, Check your id  mapping" << std::endl;
		std::cout << "exception: " << e << std::endl;
		exit(EXIT_FAILURE);
	}

	float ag_vx = new_msg->twist.twist.linear.x;
	float ag_vy = new_msg->twist.twist.linear.y;

	updated_ag_x[ag_id - 1] = ag_vx;
	updated_ag_y[ag_id - 1] = ag_vy;

	const std::lock_guard<std::mutex> lock(state_mutex);
	// Copy the value from the estimator to the state variable
	state[ag_vel_idx] = ag_vx;
	state[ag_vel_idx + 1] = ag_vy;
}

void integralStateCallback(const mavros_msgs::DebugValue::ConstPtr &debug_msg)
{
	// This function waits for debug msg with integral value and updates the state array
	// This has a special purpose re-setting the integral state if it

	// int groups[2] = {2 , 1};
	char msg_type;
	if (debug_msg->name[0] == 'I')
	{

		// Get the id and the idx from he debus msg
		int id = (int)debug_msg->data[0];
		int idx = id2idx.get_integral_idx(id, groups);

		float intgrl_x = debug_msg->data[1];
		float intgrl_y = debug_msg->data[2];

		if (std::abs(id - hrl_id) > 0.1)
		{
			// Mutex to access the state array and update the cooresponding location
			const std::lock_guard<std::mutex> lock(state_mutex);
			state[idx] = intgrl_x;
			state[idx + 1] = intgrl_y;
		}
	}
}

void resetIntegralCallback(const mavros_msgs::DebugValue::ConstPtr &debug_msg)
{
	// This function resets the value of the integral state to zero
	int trigger = (int)debug_msg->value_int;
	if (std::abs(trigger - INTRGL_RESET_TRGGR) < 0.0001)
	{
		const std::lock_guard<std::mutex> lock(state_mutex);
		for (unsigned i = 0; i < 2; i++)
		{
			intgrl_state[i] = 0.0;
		}
		ROS_INFO(" Integral state reset is complete ");
	}
}

void nextStateHRLCallback(const swarm_msgs::GlobalStates::ConstPtr &msg)
{
	const std::lock_guard<std::mutex> lock(hrl_msg_mutex);
	hrl_msg.states = msg->states;
}

void statePublisher()
{
	// This function copies the states to a seperate array and publishes to the hrl algorithm

	// This is to add a timer
	clock_t t;
	t = clock();
	ros::NodeHandle h_sp; // Handle for state publishing
	ros::Publisher state_publisher = h_sp.advertise<swarm_msgs::GlobalStates>(topic_to_hrl, 1, true);

	// Create a new msg
	swarm_msgs::GlobalStates msg;

	// Get the  array size of state ;
	//	state_mutex.lock();
	//	int sizeof_state = sizeof(state)/sizeof(state[0]);
	//	state_mutex.unlock();
	int sizeof_state = N * 6;

	// Create a copy of the state array so that we can release the lock
	float duplicate_state[sizeof_state];

	while (ros::ok())
	{

		ros::Rate loop_rate(CONTROLLER_RATE);

		// Access the lock for the states array
		state_mutex.lock();
		// Cpoy to state array to the dublicate array
		std::copy(state, state + sizeof_state, duplicate_state);

		// Release the lock
		state_mutex.unlock();

		// All states are avaibale. ready to publish
		msg.header.stamp = ros::Time::now();
		msg.states.clear();
		for (int i = 0; i < sizeof_state; i++)
		{
			// Add elememts one by one
			msg.states.push_back(duplicate_state[i]);
			// ROS_INFO(" state : %d  value : %f ", i, duplicate_state[i]);
		}
		t = clock();

		// Publish to the /swarm/global_states topic
		state_publisher.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void distToTaregtPublisher()
{
	// This function publishes the distance to the taregt
	// This does not have a thread-safe mechanism. Non-critical functions should not block the critical function 'gpsCallback'

	ros::NodeHandle h_tp; // Handle for state publishing
	ros::Publisher dist_publisher = h_tp.advertise<mavros_msgs::DebugValue>(topic_to_dist_to_target, 1, true);

	mavros_msgs::DebugValue msg;

	while (ros::ok())
	{

		ros::Rate loop_rate(CONTROLLER_RATE);

		msg.data.clear();
		msg.header.stamp = ros::Time::now();

		msg.data.resize(3);
		msg.data[0] = own_target_x;
		msg.data[1] = own_target_y;

		dist_publisher.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void integralStatelPublisher()
{
	// This function publishes the integral value from the
	mavros_msgs::DebugValue intgrl_msg;

	//	ROS_INFO(" ####### Ag 1 inside integral  published ######" );
	ros::NodeHandle nh;
	ros::Publisher intgrl_pub = nh.advertise<mavros_msgs::DebugValue>(topic_to_debug_vector, 1, true); // topic to send the integral values to other agents

	while (ros::ok())
	{

		ros::Rate loop_rate(8);
		intgrl_msg.header.stamp = ros::Time::now();

		intgrl_msg.type = 1;
		intgrl_msg.name = 'I';
		intgrl_msg.data.resize(3);

		intgrl_msg.data[0] = (float)hrl_id;
		intgrl_msg.data[1] = intgrl_state[0];
		intgrl_msg.data[2] = intgrl_state[1];

		intgrl_pub.publish(intgrl_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void init_vars(ros::NodeHandle hl)
{

	// This function assigns parameters from the config file

	// define local variables first
	float b_lat;
	float b_lon;
	int n, ng, id;
	std::vector<int> group_list, temp_id_map;
	std::vector<float> t_lat, t_lon;

	// Retrieve values from the param file
	hl.getParam("/custom_params/base_lat", b_lat);
	hl.getParam("/custom_params/base_lon", b_lon);

	hl.getParam("/custom_params/target_lat", t_lat);
	hl.getParam("/custom_params/target_lon", t_lon);

	hl.getParam("/custom_params/n_agents", n);
	hl.getParam("/custom_params/n_groups", ng);

	hl.getParam("/custom_params/groups", group_list);
	hl.getParam("/custom_params/hrl_id", id);
	hl.getParam("/custom_params/id_map", temp_id_map);

	// Assign to the global variables
	base_lat = b_lat; // Set the base.
	base_lon = b_lon;

	hrl_id = id;   // ID of the agent in the swarm. This is different from the dron ID
	n_groups = ng; // number of groups
	N = n;		   // total number of agents

	// Clusteing of the agents
	groups = new int[group_list.size()];
	//	std::array<int,group_list.size()> groups ;
	for (unsigned i = 0; i < group_list.size(); i++)
	{
		groups[i] = group_list[i];
	}

	// Assign multiple targets
	target_lat = new float[t_lat.size()];
	for (unsigned i = 0; i < t_lat.size(); i++)
	{
		target_lat[i] = t_lat[i];
	}

	target_lon = new float[t_lon.size()];
	for (unsigned i = 0; i < t_lon.size(); i++)
	{
		target_lon[i] = t_lon[i];
	}

	// Map the HRL id to the FCU id
	id_map = new int[temp_id_map.size()];
	for (unsigned i = 0; i < temp_id_map.size(); i++)
	{
		id_map[i] = temp_id_map[i];
	}

	intgrl_state = new float[2];
	for (unsigned i = 0; i < 2; i++)
	{
		intgrl_state[i] = 0.0;
	}

	// Construct state arrays that is based on the number of agents in the cluster

	state = new float[n * 6];
	for (unsigned i = 0; i < n * 6; i++)
	{
		state[i] = 0.0;
	}

	updated_ag_x = new float[n];
	updated_ag_y = new float[n];
	updated_ag_vx = new float[n];
	updated_ag_vy = new float[n];
	updated_ag_tgt_x = new float[n];
	updated_ag_tgt_y = new float[n];
}

float *positionEstimation(float x, float y, float vx, float vy, float target_x, float target_y, float t_prev, float *t_last)
{
	float t = ros::Time::now().toSec();
	float d_t = t - t_prev;

	int i, k;

	// inputs/outputs of pos, vel, and integral state
	static float arr_x[6];
	// x at t-1
	float x_last[6] = {x, y, vx, vy, target_x, target_y};
	// update position given constant velocity
	float A[6][6] = {{1, 0, d_t, 0, 0, 0}, {0, 1, 0, d_t, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, d_t, 0, 1, 0}, {0, 0, 0, d_t, 0, 1}};

	int A_rows = sizeof(A) / sizeof(A[0]);	 // 6 rows
	int A_cols = sizeof(A[0]) / sizeof(int); // 6 columns

	// Multiplying matrix A and x_last and storing in array x.
	for (i = 0; i < A_rows; ++i)
		for (k = 0; k < A_cols; ++k)
		{
			arr_x[i] += A[i][k] * x_last[k];
		}

	gps_update = false;
	*t_last = t;

	return arr_x;
}

void stateEstimater()
{
	ros::Rate loop_rate(4);
	while (ros::ok())
	{
		int i;
		float *ptr_x;
		float t_last; // not sure if i need this

		for (int j = 0; j < N; j++)
		{
			float temp_ag_x = updated_ag_x[j];
			float temp_ag_y = updated_ag_y[j];
			float temp_ag_vx = updated_ag_vx[j];
			float temp_ag_vy = updated_ag_vy[j];
			float temp_ag_tgt_x = updated_ag_tgt_x[j];
			float temp_ag_tgt_y = updated_ag_tgt_y[j];

			ptr_x = positionEstimation(temp_ag_x, temp_ag_y, temp_ag_vx, temp_ag_vy, temp_ag_tgt_x, temp_ag_tgt_y, t_prev, &t_last);

			// std::cout << "Array is: ";
			// for (i = 0; i < 6; i++)
			//     std::cout << ptr_x[i] << "\t"; // ptr[i] is equivalent to *(ptr+i)

			ag_state_idx = id2idx.get_state_idx(j+1, groups);
			ag_itgrl_idx = id2idx.get_integral_idx(j+1, groups);

			// Re-arrange stuff
			const std::lock_guard<std::mutex> lock(state_mutex);
			// Copy the value from the fcu to the state variable
			state[ag_state_idx] = ptr_x[0];
			state[ag_state_idx + 1] = ptr_x[1];

			// Copy the integral states to the state vector
			state[ag_itgrl_idx] = intgrl_state[0];
			state[ag_itgrl_idx + 1] = intgrl_state[1];

			// Update the integral state if the id is its id

			intgrl_state[0] += (ptr_x[0] - ptr_x[4]) * INTGRL_MTPL; // multiply by a scaler to reduce the magnitiude
			intgrl_state[1] += (ptr_x[1] - ptr_x[5]) * INTGRL_MTPL; // multiply by a scaller to reduce the magnitiude
		}

		if (gps_update == false)
		{
			*updated_ag_x = ptr_x[0];
			*updated_ag_y = ptr_x[1];
			*updated_ag_vx = ptr_x[2];
			*updated_ag_vy = ptr_x[3];
			*updated_ag_tgt_x = ptr_x[5];
			*updated_ag_tgt_y = ptr_x[6];
			t_prev = t_last;
		}

		// printf("value of t_last = %f", ptr_x[0]);

		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_NODE_NAME); // Initialize the node
	ros::NodeHandle hl;

	// std::string path_to_json = "/home/ubuntu/catkin_ws/src/hrl_lqi/src/org_state-matrix.json" ;// "state-matrix.json";
	init_vars(hl);

	// Path to the json file
	std::string path_to_json;
	hl.getParam("/custom_params/path_to_state_mtx", path_to_json);

	// Read the json file
	json_reader jr;
	Sys ss = jr.readJSON(path_to_json);

	// Subscribers to various topics
	ros::Subscriber gps_callback = hl.subscribe("/mavros/global_position/swarm", 100, gpsCallback);
	ros::Subscriber velocity_callback = hl.subscribe("/mavros/global_position/swarm_vel", 100, velocityCallback);
	ros::Subscriber intgrl_state_callback = hl.subscribe("/mavros/debug_value/debug_vector", 100, integralStateCallback);
	ros::Subscriber hrl_next_state_callback = hl.subscribe("/swarm/next_states/global", 100, nextStateHRLCallback);
	ros::Subscriber reset_integral_state_callback = hl.subscribe("/mavros/debug_value/named_value_int", 100, resetIntegralCallback);

	// Create seperate threads for processes with specific Hz requirements
	std::thread state_publisher(statePublisher);				   // Sperate thread for the publisher function
	std::thread dist_to_target_publisher(distToTaregtPublisher);   // Sperate thread to publish the distance to target for the agent
	std::thread integral_state_publisher(integralStatelPublisher); // Sperate thread for the publisher function
	std::thread state_estimator(stateEstimater);				   // Sperate thread for the publisher function

	// join the threads at the end
	state_publisher.join();
	dist_to_target_publisher.join();
	integral_state_publisher.join();
	state_estimator.join();

	ros::spin();

	return 0;
}
