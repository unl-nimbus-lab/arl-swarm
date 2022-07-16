/* This implements the HRL-LQI controller for the swarm project.

*/

#include <ros/ros.h>
#include <thread>
#include <string>
#include <mutex>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <math.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <custom_libs/geo_converter.hpp>
#include <custom_libs/json_reader.hpp>
#include <custom_libs/id_to_idx.hpp>
#include <swarm_msgs/GlobalStates.h>
#include <swarm_msgs/gps.h>

#define ROS_NODE_NAME "hrl_lqi"
#define CONTROLLER_RATE 4 //4   // Rate in Hz 

#define MY_ID 1

using json = nlohmann::json;
using namespace Eigen;

MatrixXd state ; // stores all the states
std::mutex state_mutex ; // mutex for state variable

id_to_idx id2idx ;

// Hackery hack ! change this when re-fectoring
float init_x = 0.0 ;
float init_y = 0.0 ;
bool init_update = true ;

float local_x = 0.0 ;
float local_y = 0.0 ;

float adj_x = 0.0 ;
float adj_y = 0.0 ;

void init_states(int n){

	// This generates a zero vector of the size of states in the experiment
	const std::lock_guard<std::mutex> lock(state_mutex);
	state = MatrixXd::Zero(n,1);
}

void preHRLCallback(const swarm_msgs::GlobalStates::ConstPtr& msg){

	// This function reads the callback messgaes, re-arrange them to the specific order of the formation

	int ag_s = msg->states.size() ;
	float temp_state[ag_s] = {0.0} ;

	const std::lock_guard<std::mutex> lock(state_mutex);
	// Copy the value from the estimator to the state variable
	for (int i=0; i<ag_s; ++i){
		auto &data = msg->states[i];
		state(i) = data ;
	}

}


void HRLController(Sys s )
{
	// This function calculates the conbtrol input to the system and propergates the system one time step forward to generate the new way points

	// Publisher topic
    ros::NodeHandle nh;
	ros::Publisher ns_pub = nh.advertise<swarm_msgs::GlobalStates> ( "/swarm/next_states/global", 1, true ); //topic for the Cube

	swarm_msgs::GlobalStates post_hrl_msg ;

	// Assign the group info from the config file
	int n_groups;
	nh.getParam("/custom_params/n_groups",n_groups);

//	int groups[1] = {1}; // This is original and works

	int groups[n_groups];
	std::vector<int> temp_groups;
	nh.getParam("/custom_params/groups",temp_groups);

	for(int i=0; i<n_groups; i++){
		groups[i]=temp_groups[i];
	}

	int ag_state_idx = id2idx.get_state_idx(MY_ID, groups); // get the index of the real agent conversion from the Transformed space


	// Create new variables for the system matrix
	MatrixXd K = s.K ;
	MatrixXd A = s.A ;
	MatrixXd B = s.B ;
	MatrixXd T = s.Tr ;
	MatrixXd aug_state = s.Aug ;
	MatrixXd target_state = s.Tgt ;

	// This commente4d code is a half-hearted effort to calculate the augmented states in the code. This prevents you to have a differet aug_state matrix pre-calculated through matlab.
/*	// Define a variable with the size equals to the number of groups
	MatrixXd aug_state(A.rows(),1);

	int aug_state_idx = id2idx.get_aug_state_idx(MY_ID, groups);
	// TODO : get the aug state from the param list. Pre-calculation saves time

        for (int i = 0, n_groups = sizeof(groups)/sizeof(groups[0]) ;  i < n_groups ; i++)
        {
		aug_state(i,0);
        }

*/

	MatrixXd T_inverse = T.inverse() ;

	MatrixXd next_states ;
	// Calculate the size of the swarm
	// TODO I thought of replacing this with states.rows() but i dont wanna access the states everytime I need n.
	int n = A.rows();

	double time_delta = 1.0/CONTROLLER_RATE ;

	while (ros::ok()) {

		ros::Rate loop_rate(CONTROLLER_RATE);

		//Access states here
		state_mutex.lock();

//		ROS_INFO(" current_states X: %f", state(ag_state_idx) );
//      ROS_INFO(" current_states Y : %f", state(ag_state_idx + 1) );

		//Calculate U here
		next_states = (A*time_delta - B*K*time_delta + MatrixXd::Identity(n,n))*(T*state-aug_state) - target_state*time_delta +aug_state ; 

		// Release the lock
		state_mutex.unlock();

		// Invert the states to get back from the transformed state
		//next_states.transposeInPlace();
		next_states = T_inverse*next_states ;

		int sizeof_next_state = next_states.size() ;

        // All states are avaibale. ready to publish
        post_hrl_msg.header.stamp = ros::Time::now();
        post_hrl_msg.states.clear();

		// Copy the elements to the array
       	for (int i = 0, nRows = next_states.rows() ;  i < nRows ; i++)
        {
			post_hrl_msg.states.push_back(next_states(i,0));
        }

		ns_pub.publish(post_hrl_msg);

		// TODO Record the states and control inputs if needed

		ros::spinOnce();
        loop_rate.sleep();

	}

}

int main( int argc, char** argv )
{
	ros::init(argc, argv, ROS_NODE_NAME); //Initialize the node

	ros::NodeHandle hrl_nd_hl;

	std::string path_to_json ;
	hrl_nd_hl.getParam("/custom_params/path_to_state_mtx",path_to_json);

	//Read the json file
	json_reader jr ;
	Sys ss = jr.readJSON(path_to_json);

	// Initialize the state
	int n = ss.A.rows();
	ROS_INFO(" N : %d", n);
	init_states(n);

    ros::Subscriber prehrl_callback = hrl_nd_hl.subscribe("/swarm/global_states", 100, preHRLCallback);

	// Create seperate threads for processes with specific Hz requirements
	std::thread hrl_controller (HRLController, ss); //Sperate thread for the controller


	// join the threads at the end
	hrl_controller.join() ;

	ros::spin();
	return 0;
}
