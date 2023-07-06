#include <iostream>
#include "ros/ros.h"
#include "safetyChecker.h"
#include "geometry_msgs/PointStamped.h"
#include "custom_libs/geo_converter.hpp"
#include "mavros_msgs/SwarmGPS.h"
#include <string>
#include <cstring>

/* This node will subscribe to the LAT/LON position of an agent and 
transform it to a cartesian frame */

safetyChecker verifier;

class agent {
    public:
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float vx = 0.0;
        float vy = 0.0;
        float vz = 0.0;
        std::tuple<hyperRectangle, float>
            computeRechableSet(
                float positionError, 
                float velocityError, 
                float reachTime, 
                float initialStepSize,
                float reachTimeout) {
                    hyperRectangle rect;
                    interval position(-positionError, positionError);
                    interval zposition(z - positionError, z + positionError);
                }

};


int main(int argc, char **argv) {

    //Initialize ROS
    ros::init(argc, argv, "verifier.cpp");
    ros::NodeHandle nh;

    //Setup Publishers and subscribers
    //ros::Subscriber pos_subscriber = nh.subscribe("/agent_position_xy", 1, &agentState::gps_cb, &thisAgent);
    //ros::Publisher pos_publisher = nh.advertise<geometry_msgs::PointStamped>("/agent_position_xy", 1, true);

    //Publish the xy position
    while( ros::ok() ) {
        //Calculate my reachable space and send it



        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    return 0;
}