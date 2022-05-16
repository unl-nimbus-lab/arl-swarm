/* This implements the HRL-LQI controller for the swarm project.

*/

//#include "lqr_control.h"


#include <ros/ros.h>
#include <thread>
#include <string>
#include <mutex>

#include <geometry_msgs/PoseStamped.h>
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
#define CONTROLLER_RATE 4    // Rate in Hz 

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
        ros::Publisher wp_pub = nh.advertise<geometry_msgs::PoseStamped> ( "/mavros/setpoint_position/local", 1, true ); //topic for the Cube

	geometry_msgs::PoseStamped pos_msg ;

	// Compute the indices
	int groups[1] = {1};

	int ag_state_idx = id2idx.get_state_idx(MY_ID, groups); // get the index of the real agent conversion from the Transformed space

	ROS_INFO(" ########### ag_state_idx  ########### : %d",ag_state_idx);

	// Create new variables for the system matrix
	MatrixXd K = s.K ;
	MatrixXd A = s.A ;
	MatrixXd B = s.B ;
	MatrixXd T = s.Tr ;
	MatrixXd aug_state = s.Aug ;
	MatrixXd target_state = s.Tgt ;

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

		// Rest of the hackery hack of setting inital vales
		if(init_update && std::abs(state(ag_state_idx)) > 0.0001){
			init_x = state(ag_state_idx);
			init_y = state(ag_state_idx+1);

			init_update = false ;
		}

		//Calculate U here
		// TODO Finish the controller implementation
		next_states = (A*time_delta - B*K*time_delta + MatrixXd::Identity(n,n))*(T*state-aug_state) - target_state*time_delta +aug_state ; 

		// Release the lock
		state_mutex.unlock();

		// Invert the states to get back from the transformed state
		//next_states.transposeInPlace();
		next_states = T_inverse*next_states ;

		// Publish to the reference topic
		pos_msg.header.stamp = ros::Time::now();
       		pos_msg.pose.position.x = next_states(ag_state_idx) - init_x ; // deduct init_x to convert back from global world to drone's local frame
        	pos_msg.pose.position.y = next_states(ag_state_idx + 1) - init_y;
		pos_msg.pose.position.z = 5 ;

		ROS_INFO(" Next pos X : %f", pos_msg.pose.position.x );
		ROS_INFO(" Next pos Y : %f", pos_msg.pose.position.y );
		ROS_INFO(" --------------------------");
		wp_pub.publish(pos_msg);

		// TODO Record the states and control inputs if needed

		ros::spinOnce();
        	loop_rate.sleep();
	}

}

int main( int argc, char** argv )
{
	ros::init(argc, argv, ROS_NODE_NAME); //Initialize the node

//	ros::NodeHandle hrl_nd_hl;
//	ros::Subscriber prehrl_callback = hrl_nd_hl.subscribe("/swarm/global_states", 100, preHRLCallback);
//        ros::Subscriber prehrl_callback = hrl_nd_hl.subscribe("/hrl/gps", 1000, preHRLCallback);

	// Path to the json file
//        std::string path_to_json = "/home/ubuntu/catkin_ws/src/hrl_lqi/src/org_state-matrix.json" ;// "sys_mats_1ag.json";
	std::string path_to_json = "/home/ubuntu/catkin_ws/src/hrl_lqi/src/sys_mats_1ag_QP.json" ;// "sys_mats_1ag.json";

	//Read the json file
	json_reader jr ;
	Sys ss = jr.readJSON(path_to_json);

	// Initialize the state
	int n = ss.A.rows();
	ROS_INFO(" N : %d", n);
	init_states(n);

        ros::NodeHandle hrl_nd_hl;
        ros::Subscriber prehrl_callback = hrl_nd_hl.subscribe("/swarm/global_states", 100, preHRLCallback);

	// Create seperate threads for processes with specific Hz requirements
	std::thread hrl_controller (HRLController, ss); //Sperate thread for the controller


	// join the threads at the end
	hrl_controller.join() ;

	ros::spin();
	return 0;
}
