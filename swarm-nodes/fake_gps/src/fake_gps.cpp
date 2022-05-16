/* This implements a fake gps to provide messages for the hrl node.

*/

#include <ros/ros.h>
#include <thread>
#include <string>
#include <mutex>
#include <mavros_msgs/DebugValue.h>
#include <time.h>
#include <std_msgs/UInt8.h>
#include "std_msgs/String.h"
#include <chrono>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>

//#include <custom_libs/geo_converter.hpp>
//#include <custom_libs/json_reader.hpp>

#include <swarm_msgs/gps.h>

#define ROS_NODE_NAME "fake_gps_node"
#define CONTROLLER_RATE 8    // Rate in Hz 
#define NN 1  // Number of agents

/* Havlock stuff
float g_lat = 40.8522108472926 ;
float g_lon = -96.60822380671102 ;
*/

/* QP stuff */
float g_lat = 40.8328860 ;
float g_lon = -96.6958694 ;

float v_x = 0.0 ;
float v_y = 0.0 ;

void StateTransmitter( )
{
	// This function outputs coordinates at a specific frequency
	ros::NodeHandle n;
	ros::Publisher gps_pub = n.advertise<swarm_msgs::gps>("/hrl/gps/", 10, true);

	double time_delta = 1/CONTROLLER_RATE ;

	while (ros::ok()) {

		ros::Rate loop_rate(CONTROLLER_RATE);

		for(int i = 1; i <= NN ; i ++){

			swarm_msgs::gps fake_msg ;
			fake_msg.header.stamp = ros::Time::now();
			fake_msg.id = i;
			fake_msg.lat = g_lat ;  //+ i*0.1;
			fake_msg.lon = g_lon ; // + i*0.1;
			fake_msg.alt = 10.0 ;
			fake_msg.vx = v_x ;
			fake_msg.vy = v_y ;
			fake_msg.vz = 0.001 ;

			gps_pub.publish(fake_msg);
			std::this_thread::sleep_for(std::chrono::nanoseconds(500000));

		}

		ros::spinOnce();
        	loop_rate.sleep();
	}


}

void trueGPSCallback(const sensor_msgs::NavSatFix msg)
{
	g_lon = msg.longitude ;
	g_lat = msg.latitude ;
//	ROS_INFO("Icoming GPS lat : %f", g_lat);
//	ROS_INFO("Icoming GPS lon : %f", g_lon);

}



void trueVelCallback(const geometry_msgs::TwistStamped msg)
{
        v_x = msg.twist.linear.x ;
        v_y = msg.twist.linear.y ;
}


void IntegralTransmitter(){

        ros::NodeHandle nh;
        ros::Publisher integral_pub = nh.advertise<mavros_msgs::DebugValue>("/mavros/debug_value/send", 10, true);

        while (ros::ok()) {

                ros::Rate loop_rate(CONTROLLER_RATE);

                for(int i = 1; i <= NN ; i ++){

                        mavros_msgs::DebugValue intgrl_msg ;
                        intgrl_msg.header.stamp = ros::Time::now();

			intgrl_msg.index = i ;
			intgrl_msg.name = "integral debug" ;
                        intgrl_msg.data.push_back((float) i) ;
			intgrl_msg.data.push_back(2.34) ;
			intgrl_msg.data.push_back(5.67) ;

			integral_pub.publish(intgrl_msg);

			std::this_thread::sleep_for(std::chrono::nanoseconds(250000));

                }

                ros::spinOnce();
                loop_rate.sleep();
        }

}

int main( int argc, char** argv )
{
	ros::init(argc, argv, ROS_NODE_NAME); //Initialize the node

	ros::NodeHandle nh;
        ros::Subscriber gps_callback = nh.subscribe("/mavros/global_position/global", 100, trueGPSCallback );
	ros::Subscriber vel_callback = nh.subscribe("/mavros/global_position/raw/gps_vel", 100, trueVelCallback );


	// Create seperate threads for processes with specific Hz requirements
	std::thread gps_transmit (StateTransmitter); //Sperate thread for transmitting GPS
//	std::thread integral_tarnsmit (IntegralTransmitter); //Sperate thread for transmitting integral states

	// join the threads at the end
	gps_transmit.join() ;
//	integral_tarnsmit.join() ;

	ros::spin();
	return 0;
}
