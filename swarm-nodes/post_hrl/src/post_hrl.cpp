
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

#define MY_ID 2 
#define CIRCLE_R 1.0
#define FIXED_HEIGHT 4

#define MAX_VEL_THRESHOLD 0.7 //was 0.7
#define MAX_OFFSET_THRESHOLD 2.5
#define MIN_OFFSET_THRESHOLD 0.5

bool tmp_flg =  false ;
bool scnd_flg = false ;

using json = nlohmann::json;
using namespace Eigen;

geo_converter gc ; // object to call gps coordinate converter functions
//id_to_idx id2idx ;

bool flag_when_mgs_published = false ;
std::mutex flag_mutex ;
std::mutex intrgl_mutex ;
std::mutex current_mutex ;

// Hackery hack ! change this when re-fectoring
// This records distance from the base of the (0,0) coordinate
//float distance_from_base_zero_x = 0.0 ;
//float distance_from_base_zero_y = 0.0 ;
//bool base_update = true ;

// Stores the coordinates where the last position msg was published
//double last_updated_x = 0.0 ;
//double last_updated_y = 0.0 ;

// Stores the current distance of the drone w.r.t. the base.
double current_x = 0.0 ;
double current_y = 0.0 ;

// records the initial local distance from the 
//bool init_local_set = true ;
//double fcu_local_distance_to_armed_x = 0.0 ; // records the distance from the power up to the start of the algorithm
//double fcu_local_distance_to_armed_y = 0.0 ;

double local_x = 0.0 ;
double local_y = 0.0 ;

double intgrl_x = 0.0 ;
double intgrl_y = 0.0 ;

// Agent current x-y distances to the target
double own_target_x = 0.0 ;
double own_target_y = 0.0 ;

int my_idx = 2 ; 

float tgty = -18.3 ;

ros::Publisher wp_pub ;
geometry_msgs::PoseStamped pos_msg ;
//mavros_msgs::Mavlink mavto_msg ;

ros::Publisher vel_pub ;
ros::Publisher vel_pos_pub ;
ros::Publisher intgrl_pub;

geometry_msgs::Twist vel_msg ;
geometry_msgs::TwistStamped vel_pos_msg ;
//mavros::plugin::SetPositionTargetLocalNEDMixin<> stmx;
//mavros::std_plugins::setpoint_posvel_plg mvplg ;

//setpoint_posvel st_pv ;

int counter = 0 ;

void init_vars(){

	// This function initialize variables for future use
	int groups[1] = {1};
	id_to_idx id2idx ;
	my_idx = id2idx.get_state_idx(MY_ID,groups);

	ros::NodeHandle nh;
      	wp_pub = nh.advertise<geometry_msgs::PoseStamped> ( "/mavros/setpoint_position/local", 1, true ); //topic for the Cube
	vel_pub = nh.advertise<geometry_msgs::Twist> ( "/mavros/setpoint_velocity/cmd_vel_unstamped", 1, true ); //topic for the Cube
	vel_pos_pub = nh.advertise<geometry_msgs::TwistStamped> ( "/mavros/setpoint_position_velocity/cmd_pos_vel", 1, true ); //topic for the Cube
	intgrl_pub = nh.advertise<mavros_msgs::DebugValue> ( "/mavros/debug_value/send", 1, true ); // topic to send the integral values to other agents

}


void preHRLCallback(const swarm_msgs::GlobalStates::ConstPtr& msg){

        // This function reads the callback messgaes to get the current position of the drone w.r.t the base

        const std::lock_guard<std::mutex> lock(current_mutex);
	current_x = msg->states[my_idx];
	current_y = msg->states[my_idx+1];

}



void HRLCallback(const swarm_msgs::GlobalStates::ConstPtr& msg){
        // This function listens to output from the hrl algorithm, checks the distance to move and publishes if the distance is beyond a threshold

	double x = msg->states[my_idx];
	double y = msg->states[my_idx+1];

	double vx = msg->states[my_idx+2];
	double vy = msg->states[my_idx+3];

	intrgl_mutex.lock();
	intgrl_x = msg->states[my_idx+4];
	intgrl_y = msg->states[my_idx+5];
	intrgl_mutex.unlock();

//	if(base_update){
//                //sets the base distance so that the net change in position after the algorithm starts executing can be detected
//                distance_from_base_zero_x = x ;
//                distance_from_base_zero_y = y ;
//                base_update = false ;
//        }

	vel_pos_msg.header.stamp = ros::Time::now();
//	pos_msg.header.stamp = ros::Time::now();
//	mavto_msg.header.stamp = ros::Time::now();


//	double distance_moved = sqrt(pow((x - distance_from_base_zero_x)-(local_x-fcu_local_distance_to_armed_x),2) + pow((y - distance_from_base_zero_y)-(local_y-fcu_local_distance_to_armed_y),2) ;
//	ROS_INFO(" distance_moved : %f", distance_moved );

	// Checks if the distance between the current location and the next state exceeds the threhold
//	if (sqrt(pow((x - distance_from_base_zero_x)-(local_x-fcu_local_distance_to_armed_x),2) + pow((y - distance_from_base_zero_y)-(local_y-fcu_local_distance_to_armed_y),2)) >= CIRCLE_R ){


//		geometry_msgs::PoseStamped pos_msg ;


		if(vx > MAX_VEL_THRESHOLD){ vx = MAX_VEL_THRESHOLD; }
		if(vx < -1*MAX_VEL_THRESHOLD){ vx = -1*MAX_VEL_THRESHOLD; }

                if(vy > MAX_VEL_THRESHOLD){ vy = MAX_VEL_THRESHOLD; }
                if(vy < -1*MAX_VEL_THRESHOLD){ vy = -1*MAX_VEL_THRESHOLD; }


//                pos_msg.pose.position.x = x - local_x;// - distance_from_base_zero_x + fcu_local_distance_to_armed_x ; // deduct distance_from_base_zero_x to get distance relative to base(0,0). Add fcu_local_distance_to_armed_xto  offset from the drone's local(0,0).
//                pos_msg.pose.position.y = y - local_y;// - distance_from_base_zero_y + fcu_local_distance_to_armed_y ;
//                pos_msg.pose.position.z = FIXED_HEIGHT ;

//		vel_msg.linear.x = vx ;
//		vel_msg.linear.y = vy ;


		// These lines are for sending the velocity and the position values to fcu
/*		vel_pos_msg.twist.linear.x = vx ;
		vel_pos_msg.twist.linear.y = vy ;

		vel_pos_msg.twist.angular.x = x ;
		vel_pos_msg.twist.angular.y = y ;
		vel_pos_msg.twist.angular.z = FIXED_HEIGHT ;
*/

//		float err = tgty - y ;

                vel_pos_msg.twist.linear.x = vx ;
                vel_pos_msg.twist.linear.y = vy ;
		vel_pos_msg.twist.linear.y = 0.0 ;

		float target_sign_x= 1 ;
		float target_sign_y= 1 ;

		if(own_target_x < 0){target_sign_x= -1;}
		if(own_target_y < 0){target_sign_y= -1;}

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


		current_mutex.lock();
                vel_pos_msg.twist.angular.x = x - current_x + offset_x ;// - local_x ; // angular for position
                vel_pos_msg.twist.angular.y = y - current_y + offset_y;
                vel_pos_msg.twist.angular.z = 0.0 ;

		current_mutex.unlock();

//                pos_msg.pose.position.x = x - current_x + offset_x ;
//                pos_msg.pose.position.y = y - current_y + offset_y ;


//		vel_msg.linear.x = 0.0 ;
//               vel_msg.linear.y = -1.0 ;

//		ROS_INFO(" pos+vel msg to fcu to y direction: %f", vel_pos_msg.twist.angular.y );


//		ROS_INFO(" To pos+vel x - local_x : %f , y - local_y : %f , vx : %f , vy : %f   ", x - local_x, y - local_y, vx, vy );


                ROS_INFO(" x - current_x : %f", x - current_x );
                ROS_INFO(" y - current_y : %f", y - current_y );

                ROS_INFO(" vx : %f", vx );
                ROS_INFO(" vy : %f", vy );

                ROS_INFO(" vel_pos_msg.twist.angular.x : %f", vel_pos_msg.twist.angular.x );
                ROS_INFO(" vel_pos_msg.twist.angular.y : %f", vel_pos_msg.twist.angular.y );

                ROS_INFO(" own_target_x : %f", own_target_x );
                ROS_INFO(" own_target_y : %f", own_target_y );

                ROS_INFO(" offset_x : %f", offset_x );
                ROS_INFO(" offset_y : %f", offset_y );

/*		mavto_msg.payload64 = push_back(x);
		mavto_msg.payload64 = push_back(y);
		mavto_msg.payload64 = push_back(z);

		mavto_msg.payload64 = push_back(vx);
		mavto_msg.payload64 = push_back(vy);
		mavto_msg.payload64 = push_back(0.0);
*/


//                ROS_INFO(" fcu_local_distance_to_armed_x : %f", fcu_local_distance_to_armed_x );
//                ROS_INFO(" fcu_local_distance_to_armed_y : %f", fcu_local_distance_to_armed_y );

//                ROS_INFO(" distance_from_base_zero_x : %f", distance_from_base_zero_x );
//                ROS_INFO(" distance_from_base_zero_y : %f", distance_from_base_zero_y );

//		ROS_INFO(" pos_msg.pose.position.x : %f", pos_msg.pose.position.x );
//		ROS_INFO(" pos_msg.pose.position.y : %f", pos_msg.pose.position.y );

		// Send a signal when the hrl distance exceeds the threshold.
//		flag_mutex.lock();
//		flag_when_mgs_published = true ;
//		flag_mutex.unlock();
//	}

//	wp_pub.publish(vel_pos_msg);
//	vel_pos_pub.publish(vel_pos_msg);
//	vel_pub.publish(vel_msg);

/*	scnd_flg = true ;
	if(tmp_flg && scnd_flg){
		vel_pos_pub.publish(vel_pos_msg);
		tmp_flg = false;
//		scnd_flg = false ;
	}
*/


//	wp_pub.publish(pos_msg);
	vel_pos_pub.publish(vel_pos_msg);

//	st_pv.send_setpoint_pos_velocity(pos_msg.header.stamp.nsec/100, 3,0.0)
//	if(counter == CONTROLLER_RATE/2){
//		ROS_INFO(" $$$$ vel msg published $$$$$ ");
//		counter = 0 ;
//		vel_pub.publish(vel_msg);
//	}
//	counter++ ;

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


/*
void localPosCallback(const nav_msgs::Odometry::ConstPtr& msg){
	// This function reads local positon messages and records the local position in x,y directions
	if(init_local_set){
		//sets the initial distance so that the net change in position after the algorithm starts executing can be detected
	        fcu_local_distance_to_armed_x = msg->pose.pose.position.x ;
        	fcu_local_distance_to_armed_y = msg->pose.pose.position.y ;
		init_local_set = false ;
	}

	local_x = msg->pose.pose.position.x ;
	local_y = msg->pose.pose.position.y ;

}
*/

void distToTgTCallback(const mavros_msgs::DebugValue::ConstPtr& msg){
	// Records current state values

	own_target_x = msg->data[0];
	own_target_y = msg->data[1];

}
/*
void signalPublisher(){
	// This function is created to send a signal to a different thread when a msg is published but it's not been used now

        ros::NodeHandle h_phrl ; // Handle for state publishing
        ros::Publisher signal_publisher = h_phrl.advertise<swarm_msgs::signal> ( "/swarm/signal/local" , 1, true );

        // Create a new msg
        swarm_msgs::signal msg ;

        while (ros::ok()) {

                ros::Rate loop_rate(CONTROLLER_RATE);

		msg.header.stamp = ros::Time::now();
		flag_mutex.lock();
		if(flag_when_mgs_published == true){
 	               msg.signal = true;
			flag_when_mgs_published = false ;
			 ROS_INFO(" ####### true signal published ######" );
		}else
		{
			msg.signal = false ;
		}
                flag_mutex.unlock();

		signal_publisher.publish(msg);

                ros::spinOnce();
                loop_rate.sleep();
        }

}
*/

void integralPublisher(){
	// This function publishes the integral value from the 
	mavros_msgs::DebugValue intgrl_msg ;

	ROS_INFO(" ####### inside integral  published ######" );

        while (ros::ok()) {

                ros::Rate loop_rate(CONTROLLER_RATE);
		intgrl_msg.header.stamp = ros::Time::now();

		intgrl_msg.type = 1 ;
		intgrl_msg.name = 'I';
		intgrl_msg.data.resize(3);

		intgrl_msg.data[0] = (float) my_idx ;
	        intrgl_mutex.lock();
        	intgrl_msg.data[1] = intgrl_x ;
		intgrl_msg.data[2] = intgrl_y ;
        	intrgl_mutex.unlock();

		intgrl_pub.publish(intgrl_msg);

		ros::spinOnce();
                loop_rate.sleep();

	}



}

int main( int argc, char** argv )
{
	ros::init(argc, argv, ROS_NODE_NAME); //Initialize the node

	init_vars();

        ros::NodeHandle post_hrl_hl;
        ros::Subscriber hrl_callback = post_hrl_hl.subscribe("/swarm/next_states/global", 100, HRLCallback);
//	ros::Subscriber local_pos_callback = post_hrl_hl.subscribe("/mavros/global_position/local", 100, localPosCallback);
//	 ros::Subscriber rc_callback = post_hrl_hl.subscribe("/mavros/rc/in", 100, rcCallback);
        ros::Subscriber prehrl_callback = post_hrl_hl.subscribe("/swarm/global_states", 100, preHRLCallback);
        ros::Subscriber dist_to_tgt_callback = post_hrl_hl.subscribe("/swarm/dist_to_target/", 100, distToTgTCallback);

  //      std::thread signal_publisher (signalPublisher); //Sperate thread for the publisher function

	std::thread integral_publisher (integralPublisher); //Sperate thread for the integral publisher function

       // join the threads at the end
    //    signal_publisher.join() ;
	integral_publisher.join();

	ros::spin();
	return 0;
}
