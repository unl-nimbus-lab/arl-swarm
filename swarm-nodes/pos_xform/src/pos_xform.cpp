#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "custom_libs/geo_converter.hpp"
#include "mavros_msgs/SwarmGPS.h"
#include <string>
#include <cstring>

/* This node will subscribe to the LAT/LON position of an agent and 
transform it to a cartesian frame */

geo_converter converter;

double sign(double input) {
    //Returns the direction of a double
    return (input < 0.0) ? -1.0 : 1.0;
}

class agentState {
    public:
        int SYSID_THISMAV = 1;
        double ORIGIN_LAT = 0.0;
        double ORIGIN_LON = 0.0;
        double X_POS= 0.0;
        double Y_POS = 0.0;

        void gps_cb(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
            //Callback to update the X/Y_POS data in the state class
            double xSign = sign(msg->latitude - ORIGIN_LAT);
            double ySign = sign(msg->longitude - ORIGIN_LON);

            SYSID_THISMAV = msg->system_id;

            Y_POS = ySign * converter.gps_to_meters(ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT, msg->longitude);
            X_POS = xSign * converter.gps_to_meters(ORIGIN_LAT, ORIGIN_LON, msg->latitude, ORIGIN_LON);
        }
};

int main(int argc, char **argv) {
    agentState thisAgent;

    thisAgent.SYSID_THISMAV = atoi(argv[1]);
    thisAgent.ORIGIN_LAT = std::stod(argv[2]);
    thisAgent.ORIGIN_LON = std::stod(argv[3]);

    //Initialize ROS
    ros::init(argc, argv, "pos_xform");
    ros::NodeHandle nh;

    //Setup Publishers and subscribers
    ros::Subscriber gps_subscriber = nh.subscribe("/mavros/global_position/swarm", 1, &agentState::gps_cb, &thisAgent);
    ros::Publisher pos_publisher = nh.advertise<geometry_msgs::PointStamped>("/agent_position_xy", 1, true);

    //ROS convert the position here here
    return 0;
}

