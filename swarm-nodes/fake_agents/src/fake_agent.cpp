#include <iostream>
#include "ros/ros.h"
#include "safetyChecker.h"
#include "geometry_msgs/PointStamped.h"
#include "custom_libs/FullState.h"
#include "custom_libs/geo_converter.hpp"
#include "mavros_msgs/SwarmGPS.h"
#include "verifier_comms/hyperRectangle.h"
#include "mavros_msgs/SetMode.h"
#include <string>
#include <cstring>
#include <fstream>


int main(int argc, char **argv) {

    int thisAgent = 3;


    //Initialize ROS
    ros::init(argc, argv, "fakeAgent");
    ros::NodeHandle nh;

    //Setup Publishers and subscribers
    ros::Publisher pos_publisher = nh.advertise<verifier_comms::hyperRectangle>("/reachable_state", 1, true);

    verifier_comms::hyperRectangle rsMessage;
    rsMessage.id = thisAgent;

    while( ros::ok() ) {

        rsMessage.x.min = -20;
        rsMessage.x.max = 20;

        rsMessage.y.min = -20;
        rsMessage.y.max = 20;

        rsMessage.z.min = 300;
        rsMessage.z.max = 400;

        rsMessage.vx.min = -1;
        rsMessage.vx.max = 1;

        rsMessage.vy.min = -1;
        rsMessage.vy.max = 1;

        rsMessage.vz.min = -1;
        rsMessage.vz.max = 1;

        rsMessage.header.stamp = ros::Time::now();

        pos_publisher.publish(rsMessage);


        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    return 0;
}