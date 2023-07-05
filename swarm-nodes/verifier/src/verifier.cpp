#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "custom_libs/geo_converter.hpp"
#include "mavros_msgs/SwarmGPS.h"
#include <string>
#include <cstring>

/* This node will subscribe to the LAT/LON position of an agent and 
transform it to a cartesian frame */


int main(int argc, char **argv) {

    //Initialize ROS
    ros::init(argc, argv, "verifier.cpp");
    ros::NodeHandle nh;

    //Setup Publishers and subscribers
    //ros::Subscriber gps_subscriber = nh.subscribe("/mavros/global_position/swarm", 1, &agentState::gps_cb, &thisAgent);
    //ros::Publisher pos_publisher = nh.advertise<geometry_msgs::PointStamped>("/agent_position_xy", 1, true);

    //Publish the xy position
    while( ros::ok() ) {
        //Calculate my reachable space and publish it



        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    return 0;
}