
/* This implements preparation of the complete state space for the HRL-LQI controller.
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
#include <chrono>
#include <time.h>

#define ROS_NODE_NAME "pre_hrl"
#define CONTROLLER_RATE 8    // Rate in Hz

#define MY_ID 1

#define INTGRL_MTPL 0.001 // A scalar multiplier to ewduce the magnitude of the integral state

float state[6] = {0.0} ; // stores all the states
float intgrl_state[2] = {0.0} ;
std::mutex state_mutex ; // mutex for state variable

geo_converter gc ; // object to call gps coordinate converter functions
id_to_idx id2idx ;

int groups[1] = {1};

swarm_msgs::GlobalStates hrl_msg ;
std::mutex hrl_msg_mutex ;

bool use_gps = true ;
std::mutex signal_mutex ;


/* Havlock stuff
float base_lat = 40.8522108472926 ;
float base_lon = -96.60822380671102 ;

float target_lat = 40.852158006872756 ;
float target_lon = -96.60885620373058 ;
*/

/* QP stuff */
float base_lat = 40.8328860 ;
float base_lon = -96.6958694 ;

float target_lat = 40.8327215 ;
float target_lon = -96.6958784 ;


std::string topic_to_hrl = "/swarm/global_states/" ;

void gpsCallback(swarm_msgs::gps new_msg ){

	// This function reads the callback messgaes, re-arrange them to the specific order of the formation
	int ag_id = new_msg.id ;

	// Get the corresponding index in the transformed state space given an agent id
	int ag_state_idx = id2idx.get_state_idx(ag_id, groups);
	int ag_itgrl_idx = id2idx.get_integral_idx(ag_id, groups);

	// Calculate state values from the new msg
	float ag_x = gc.lon_to_meters_signed(base_lat,base_lon, base_lat,new_msg.lon) ;// x state with respect to the base (0,0)
	float ag_y = gc.lat_to_meters_signed(base_lat,base_lon, new_msg.lat,base_lon) ;// y state with respect to the base (0,0)
	float ag_vx = new_msg.vx ;
	float ag_vy = new_msg.vy ;

	// Update the integral state if the id is its id
	if ( std::abs(ag_id - MY_ID) < 0.000000001 ){

		float ag_tgt_x = gc.lon_to_meters_signed(base_lat,base_lon, base_lat,target_lon) ;// x distance to taregt with respect to the base (0,0)
		float ag_tgt_y = gc.lat_to_meters_signed(base_lat,base_lon, target_lat,base_lon) ;// y distance to taregt with respect to the base (0,0)  
		intgrl_state[0] += (ag_x - ag_tgt_x)*INTGRL_MTPL ; // multiply by a scaler to reduce the magnitiude 
		intgrl_state[1] += (ag_y - ag_tgt_y)*INTGRL_MTPL ; // multiply by a scaller to reduce the magnitiude 
	}


	// Re-arrange stuff
	const std::lock_guard<std::mutex> lock(state_mutex);
	// Copy the value from the estimator to the state variable
	state[ag_state_idx] = ag_x ;
	state[ag_state_idx+1] = ag_y ;
	state[ag_state_idx+2] = ag_vx ;
	state[ag_state_idx+3] = ag_vy ;

	// Copy the integral states to the state vector
	if ( std::abs(ag_id - MY_ID) < 0.000000001 ){
		state[ag_itgrl_idx] = intgrl_state[0];
		state[ag_itgrl_idx+1] = intgrl_state[1];
	}

}


void integralStateCallback(mavros_msgs::DebugValue debug_msg){
	// This function waits for debug msg with integral value and updates the state array

	//int groups[2] = {2 , 1};

	// Get the id and the idx from he debus msg
	int id = (int) debug_msg.data[0];
	int idx = id2idx.get_integral_idx(id, groups);

	float intgrl_x = debug_msg.data[1] ;
	float intgrl_y = debug_msg.data[2] ;

	// Mutex to access the state array and update the cooresponding location 
	const std::lock_guard<std::mutex> lock(state_mutex);
	state[idx] = intgrl_x ;
	state[idx+1] = intgrl_y ;
}


void nextStateHRLCallback(const swarm_msgs::GlobalStates::ConstPtr& msg)
{
	const std::lock_guard<std::mutex> lock(hrl_msg_mutex);
	hrl_msg.states = msg->states;
}

void signalCallback(const swarm_msgs::signal::ConstPtr& msg)
{
	const std::lock_guard<std::mutex> lock(signal_mutex);
	use_gps = msg -> signal ;
}


void statePublisher()
{
	// This function copies the states to a seperate array and publishes to the hrl algorithm

	// This is to add a timer
	clock_t t ;
	t = clock() ;
	ros::NodeHandle h_sp ; // Handle for state publishing
	ros::Publisher state_publisher = h_sp.advertise<swarm_msgs::GlobalStates> ( topic_to_hrl , 1, true );

	// Create a new msg
	swarm_msgs::GlobalStates msg ;

	// Get the  array size of state ;
	state_mutex.lock();
	int sizeof_state = sizeof(state)/sizeof(state[0]);
	state_mutex.unlock();

	// Create a copy of the state array so that we can release the lock
	float duplicate_state[sizeof_state];

	while (ros::ok()) {

		ros::Rate loop_rate(CONTROLLER_RATE);

		// Access the lock for the states array
		state_mutex.lock();
		// Cpoy to state array to the dublicate array
		std::copy(state, state+sizeof_state, duplicate_state);

		// Release the lock
                state_mutex.unlock();

		// All states are avaibale. ready to publish
		msg.header.stamp = ros::Time::now();
		msg.states.clear();
		for(int i = 0; i < sizeof_state ; i++){
			// Add elememts one by one
			msg.states.push_back(duplicate_state[i]);
			//ROS_INFO(" state : %d  value : %f ", i, duplicate_state[i]);

		}

			t = clock() ;

		// Publish to the /swarm/global_states topic
		state_publisher.publish(msg);

		ros::spinOnce();
        	loop_rate.sleep();
	}

}

int main( int argc, char** argv )
{
	ros::init(argc, argv, ROS_NODE_NAME); //Initialize the node

        // Path to the json file
        std::string path_to_json = "/home/ubuntu/catkin_ws/src/hrl_lqi/src/org_state-matrix.json" ;// "state-matrix.json";

        //Read the json file
        json_reader jr ;
        Sys ss = jr.readJSON(path_to_json);

	ros::NodeHandle hl;
	ros::Subscriber gps_callback = hl.subscribe("/hrl/gps", 100, gpsCallback );
	ros::Subscriber intgrl_state_callback = hl.subscribe("/mavros/debug_value/send", 100, integralStateCallback);
	ros::Subscriber hrl_next_state_callback = hl.subscribe( "/swarm/next_states/global", 100, nextStateHRLCallback );
	ros::Subscriber signal_callback = hl.subscribe("/swarm/signal/local", 100, signalCallback );

	// Create seperate threads for processes with specific Hz requirements
	std::thread state_publisher (statePublisher); //Sperate thread for the publisher function

	// join the threads at the end
	state_publisher.join() ;

	ros::spin();

	return 0;
}
