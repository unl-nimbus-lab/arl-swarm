
/* This function does two things for the swarm project. However, none of the objectives are implemented due to various isses with arducopter
1. Checks if the next location to move is within a threshold
2. If the threshold exceeds, a setpoint_local command is published to the fcu

What it does is it publishes either
1. A position only waypoint to setpoint_position topic
2. A velocity only waypoint to setpoint_velocity topic
3. Both position and velocity to the respective topics interchangably
*/

#include <ros/ros.h>
#include <thread>
#include <string>
#include <mutex>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/RCIn.h>
#include "std_msgs/String.h"
#include <math.h>
#include <limits>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <custom_libs/geo_converter.hpp>
#include <custom_libs/json_reader.hpp>
#include <custom_libs/id_to_idx.hpp>
//#include <custom_libs/setpoint_posvel.hpp>
#include <swarm_msgs/GlobalStates.h>
#include <swarm_msgs/gps.h>
#include <swarm_msgs/signal.h>
#include <mavros_msgs/DebugValue.h>
#include <swarm_msgs/GlobalStates.h>
//#include <mavros_msgs/Mavlink.h>


//#include <mavros/setpoint_mixin.h>
//#include <mavros/mavros_plugin.h>

//setpoint_posvel_plg

#define ROS_NODE_NAME "post_hrl"
#define CONTROLLER_RATE 50    // Rate in Hz 

//#define MY_ID 2 
#define CIRCLE_R 1.0 // Radius of the gps error
#define FIXED_HEIGHT 4

#define MAX_VEL_THRESHOLD 0.7 // limit the maximum velocity of the UAS
#define MAX_OFFSET_THRESHOLD 2.5 // Add this amount as max-offset to the distance calculated by the algo
#define MIN_OFFSET_THRESHOLD 0.5 // Add this amount as min-offset to the distance calculated by the algo

bool tmp_flg =  false ;
bool scnd_flg = false ;

using json = nlohmann::json; // Create an object for parsing the JSON matrix
using namespace Eigen;

geo_converter gc ; // object to call gps coordinate converter functions

bool flag_when_mgs_published = false ;
std::mutex flag_mutex ;
std::mutex intrgl_mutex ;
std::mutex current_mutex ;

int hrl_id ; // ID of the agent in the swarm. This is different from the dron ID
int* groups = NULL; // vector to hold the agent clusters in the swarm

// Stores the current distance of the drone w.r.t. the base.
double current_x = 0.0 ;
double current_y = 0.0 ;

double local_x = 0.0 ;
double local_y = 0.0 ;

double intgrl_x = 0.0 ;
double intgrl_y = 0.0 ;

// Agent current x-y distances to the target
double own_target_x = 0.0 ;
double own_target_y = 0.0 ;

int my_pos_state_idx ; 
int my_vel_state_idx ;

ros::Publisher wp_pub ;
geometry_msgs::PoseStamped pos_msg ;

ros::Publisher vel_pub ;
ros::Publisher vel_pos_pub ;

geometry_msgs::Twist vel_msg ;
geometry_msgs::TwistStamped vel_pos_msg ;


int counter = 0 ;

void init_vars(ros::NodeHandle hl){

	// This function initialize variables for future use

        // Assign values from the param file
        int id;
        hl.getParam("/custom_params/hrl_id",id);
        hrl_id = id ; // ID of the agent in the swarm. This is different from the dron ID

	std::vector<int> group_list;
	hl.getParam("/custom_params/groups",group_list);

        // Clusteing of the agents
        groups = new int[group_list.size()];
        for(unsigned i=0; i < group_list.size(); i++) {
               groups[i] = group_list[i] ;
        }

//	int groups[1] = {1};
	id_to_idx id2idx ;
	my_pos_state_idx = id2idx.get_state_idx(hrl_id,groups);
	my_vel_state_idx = id2idx.get_velocity_idx(hrl_id, groups);

	ros::NodeHandle nh;
      	wp_pub = nh.advertise<geometry_msgs::PoseStamped> ( "/mavros/setpoint_position/local", 1, true ); //topic for the Cube
	vel_pub = nh.advertise<geometry_msgs::Twist> ( "/mavros/setpoint_velocity/cmd_vel_unstamped", 1, true ); //topic for the Cube
	vel_pos_pub = nh.advertise<geometry_msgs::TwistStamped> ( "/mavros/setpoint_position_velocity/cmd_pos_vel", 1, true ); //topic for the Cube

}


void preHRLCallback(const swarm_msgs::GlobalStates::ConstPtr& msg){

        // This function reads the callback messgaes to get the current position of the drone w.r.t the base

        const std::lock_guard<std::mutex> lock(current_mutex);
	current_x = msg->states[my_pos_state_idx];
	current_y = msg->states[my_pos_state_idx+1];

}



void HRLCallback(const swarm_msgs::GlobalStates::ConstPtr& msg){
        // This function listens to output from the hrl algorithm, checks the distance to move and publishes if the distance is beyond a threshold

	// Copy messages from the states passed from hrl_lqi


	double x = msg->states[my_pos_state_idx];
	double y = msg->states[my_pos_state_idx+1];

	double vx = msg->states[my_vel_state_idx];
	double vy = msg->states[my_vel_state_idx+1];

/*	intrgl_mutex.lock();
	intgrl_x = msg->states[my_idx+4];
	intgrl_y = msg->states[my_idx+5];
	intrgl_mutex.unlock();
*/
	vel_pos_msg.header.stamp = ros::Time::now();

	// Apply thresholds to the velocity
	if(vx > MAX_VEL_THRESHOLD){ vx = MAX_VEL_THRESHOLD; }
	if(vx < -1*MAX_VEL_THRESHOLD){ vx = -1*MAX_VEL_THRESHOLD; }

        if(vy > MAX_VEL_THRESHOLD){ vy = MAX_VEL_THRESHOLD; }
        if(vy < -1*MAX_VEL_THRESHOLD){ vy = -1*MAX_VEL_THRESHOLD; }


        // Record the velocity values in the message
        vel_pos_msg.twist.linear.x = vx ;
        vel_pos_msg.twist.linear.y = vy ;

	// Next lines add thresholds to the position values
	//Determine the direction of the offset to be added
	float target_sign_x= 1 ;
	float target_sign_y= 1 ;

	if(own_target_x < 0){target_sign_x= -1;}
	if(own_target_y < 0){target_sign_y= -1;}

	// Calculate the offset to be (x^2)/2 
	float offset_x = (own_target_x)*(own_target_x)*0.5*target_sign_x;
	float offset_y = (own_target_y)*(own_target_y)*0.5*target_sign_y;

	if(own_target_x > MAX_OFFSET_THRESHOLD ){ offset_x = MAX_OFFSET_THRESHOLD ;}
	if(own_target_x < MIN_OFFSET_THRESHOLD && own_target_x >= 0.0 ) {offset_x = 0.0;}

	if(own_target_x < -1*MAX_OFFSET_THRESHOLD ){ offset_x = -1*MAX_OFFSET_THRESHOLD ;}
        if(own_target_x > -1*MIN_OFFSET_THRESHOLD && own_target_x <= 0.0 ) {offset_x = 0.0;}


        if(own_target_y > MAX_OFFSET_THRESHOLD ){ offset_y = MAX_OFFSET_THRESHOLD ;}
        if(own_target_y < MIN_OFFSET_THRESHOLD && own_target_y >= 0.0 ) {offset_y = 0.0;}

        if(own_target_y < -1*MAX_OFFSET_THRESHOLD ){ offset_y = -1*MAX_OFFSET_THRESHOLD ;}
        if(own_target_y > -1*MIN_OFFSET_THRESHOLD && own_target_y <= 0.0 ) {offset_y = 0.0;}


        // Add the offsets
	current_mutex.lock();
        vel_pos_msg.twist.angular.x = x - current_x + offset_x ;// - local_x ; // angular for position
        vel_pos_msg.twist.angular.y = y - current_y + offset_y;
        vel_pos_msg.twist.angular.z = 0.0 ;

	current_mutex.unlock();

//		ROS_INFO(" To pos+vel x - local_x : %f , y - local_y : %f , vx : %f , vy : %f   ", x - local_x, y - local_y, vx, vy );

	// Publish the message
	vel_pos_pub.publish(vel_pos_msg);

}

void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg){

	int switch_val = (unsigned int)msg->channels[6];
//	ROS_INFO(" radio RC input: %d", switch_val);

	if(switch_val > 1510){
		tmp_flg = true ;
	}else{
		tmp_flg = false ;
		scnd_flg = true ;
	}

}


void distToTgTCallback(const mavros_msgs::DebugValue::ConstPtr& msg){
	// Records current state values
	own_target_x = msg->data[0];
	own_target_y = msg->data[1];

}


int main( int argc, char** argv )
{
	ros::init(argc, argv, ROS_NODE_NAME); //Initialize the node

	ros::NodeHandle post_hrl_hl;
	init_vars(post_hrl_hl);

        ros::Subscriber hrl_callback = post_hrl_hl.subscribe("/swarm/next_states/global", 100, HRLCallback);
//	 ros::Subscriber rc_callback = post_hrl_hl.subscribe("/mavros/rc/in", 100, rcCallback);
        ros::Subscriber prehrl_callback = post_hrl_hl.subscribe("/swarm/global_states", 100, preHRLCallback);
        ros::Subscriber dist_to_tgt_callback = post_hrl_hl.subscribe("/swarm/dist_to_target/", 100, distToTgTCallback);


	ros::spin();
	return 0;
}
