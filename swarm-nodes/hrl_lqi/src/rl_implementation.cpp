/* This file is to test velocity msg for swarm project*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <thread>
#include <string>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <math.h>
#include <algorithm>
#include "std_msgs/Byte.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <tf/tf.h>
#include <time.h>
#include "json.hpp"
#include <fstream>
#include <eigen-master/Eigen/Dense>

//Parameters for consensus algorithm

#define WP_RATE 30  // Rate to publish the waypoints to the pixhawk at rate > 3Hz

#define LNK_FLATEARTH_X 111050.51 //Flat earth assumption constants for Lincoln area to get distance between two GPS points
#define LNK_FLATEARTH_Y 84356.28

#define EARTH_R 6378137 // This is the radius of the earth in meters

uint8_t ignore_rates = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;

using json = nlohmann::json;
using namespace Eigen;

void publishToWaypoints(){

	ROS_INFO("Velocity thread started");

	geometry_msgs::Twist VelMsg ;

//        VelMsg.header.stamp = ros::Time::now();
	VelMsg.linear.x = 1.0 ;
	VelMsg.linear.y = 0.0 ;
	VelMsg.linear.z = 0.0 ;

	VelMsg.angular.x = 0.5 ;

	ros::NodeHandle VelHandle ;
	ros::Publisher VelPublisher = VelHandle.advertise<geometry_msgs::Twist> ( "/mavros/setpoint_velocity/cmd_vel_unstamped", 1, true ); //topic for pixhawk controller

	while (ros::ok()) {
		ros::Rate loop_rate(WP_RATE);
		VelPublisher.publish(VelMsg);
		ros::spinOnce();
        	loop_rate.sleep();
	}
}


/*
void publishToWaypoints(){

        ROS_INFO("Acceleration thread started");

        geometry_msgs::Vector3Stamped AccMsg ;

        AccMsg.header.stamp = ros::Time::now();
        AccMsg.vector.x = -1.0 ;
        AccMsg.vector.y = 0.0 ;
        AccMsg.vector.z = 0.0 ;


        ros::NodeHandle AccHandle ;
        ros::Publisher AccPublisher = AccHandle.advertise<geometry_msgs::Vector3Stamped> ( "/mavros/setpoint_accel/accel", 1, true );

        while (ros::ok()) {
                ros::Rate loop_rate(WP_RATE);
                AccPublisher.publish(AccMsg);
                //ROS_INFO("Acceleration msg published");
                ros::spinOnce();
                loop_rate.sleep();
        }
}
*/

/*
void publishToWaypoints(){

	ROS_INFO("Inner thread started");

	//Construct the roll pitch yaw thrust

	double target_pitch = 0.0 ;
	double target_roll = 0.0 ;
	double target_yawrate = 0.0 ;
	double target_thrust = 0.47 ;

	target_pitch = std::max( -1.0, std::min( 1.0, target_pitch ) );
	target_roll = std::max( -1.0, std::min( 1.0, target_roll ) );
	target_yawrate = std::max( -45.0, std::min( 45.0, target_yawrate ) );
	target_thrust = std::max( THRUST_MIN, std::min( THRUST_MAX, target_thrust ) );


 	geometry_msgs::Quaternion target_q;
	target_q = tf::createQuaternionMsgFromRollPitchYaw( target_roll, target_pitch, target_yawrate );

	mavros_msgs::AttitudeTarget att_msg ;

	att_msg.type_mask = ignore_rates;
	att_msg.orientation = target_q;

	att_msg.thrust = target_thrust;
  	att_msg.body_rate.z = -target_yawrate;
	att_msg.header.stamp = ros::Time::now();
	att_msg.header.frame_id = "swarm_id" ;

	ros::NodeHandle node_handler ;
	ros::Publisher attitude_publisher = node_handler.advertise <mavros_msgs::AttitudeTarget> ( "/mavros/setpoint_raw/attitude", 1, true );


	double roll_array[] = {0.2 , 0.0 , -0.2 , 0.0 };
	double pitch_array[] = {0.0 , -0.2 , 0.0 , -0.2};

	time_t timer1;
	time_t timer2;

	int i = 0 ;

	time(&timer1);
        while (ros::ok()) {
                ros::Rate loop_rate(WP_RATE);

		if(difftime(timer2,timer1) > 1){

			target_q = tf::createQuaternionMsgFromRollPitchYaw( roll_array[i%3], pitch_array[i%3], target_yawrate );
			i++ ;
			att_msg.orientation = target_q;
			time(&timer1);
			ROS_INFO("Orientation Changed");
		}

		time(&timer2);

		attitude_publisher.publish(att_msg);
                //ROS_INFO("Attiude  msg published Diff time %f", difftime(timer1,timer2));
                ros::spinOnce();
                loop_rate.sleep();
        }


}
*/


void initParams(std::string path_to_json){

	std::ifstream i(path_to_json);
	json j;
	i >> j;

	auto TA_matrix = j["TA"] ;
	auto TB_matrix = j["TB"] ;


	int TA_size =  TA_matrix.size() ;
	int TB_size =  TB_matrix.size() ;

	MatrixXd A_matrix(TA_size,TA_size);
	MatrixXd B_matrix(TB_size,TB_size);

	// Assign each value for A matrix to Eigen matrix
	for (auto& el : TA_matrix.items())
	{
		int i = 0 ;
		for (auto& subel : el.value()){
			A_matrix(stoi(el.key()),i) = subel ;
			i++ ;
		}
	}
	std::cout << A_matrix ;


        // Assign each value for B matrix to Eigen matrix
        for (auto& el : TB_matrix.items())
        {
                int i = 0 ;
                for (auto& subel : el.value()){
                        B_matrix(stoi(el.key()),i) = subel ;
                        i++ ;
                }
        }

        std::cout << B_matrix  ;



}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "communicator"); //Initialize the node

        std::string path_to_json = "/home/odroid/state-matrix.json";

	initParams(path_to_json);


	ros::NodeHandle inHandle;

	ROS_INFO("ROS Started");

//	std::thread publisher (publishToOutbound); //Sperate thread for the listener
	std::thread waypointPublisher(publishToWaypoints); //Sperate thread for the listener

	ros::spin();

	//Join the two threads
	waypointPublisher.join();

	return 0;
}


