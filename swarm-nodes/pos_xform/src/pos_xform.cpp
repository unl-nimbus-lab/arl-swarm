#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "custom_libs/geo_converter.hpp"
#include "mavros_msgs/SwarmGPS.h"
#include <string>
#include <cstring>
#include "custom_libs/FullState.h"

/* This node will subscribe to the LAT/LON position of an agent and 
transform it to a cartesian frame */

geo_converter converter;

double sign(double input) {
    //Returns the sign of a double
    return (input < 0.0) ? -1.0 : 1.0;
}

class agentState {
    //Class to maintain agent state
    public:
        int SYSID_THISMAV = 1;
        double ORIGIN_LAT = 0.0;
        double ORIGIN_LON = 0.0;
        double X_POS= 0.0;
        double Y_POS = 0.0;
        double Z_POS = 0.0;
        double X_VEL = 0.0;
        double Y_VEL = 0.0;
        double Z_VEL = 0.0;
        double X_ACC = 0.0;
        double Y_ACC = 0.0;
        double Z_ACC = 0.0;
        int lastSec = 0;
        int lastNsec = 0;


        void gps_cb(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
            //Callback to update the X/Y_POS data in the state class
            
            if (msg->system_id == SYSID_THISMAV) {

                double oldXPos = X_POS;
                double oldYPos = Y_POS;
                double oldZPos = Z_POS;
                double oldXVel = X_VEL;
                double oldYVel = Y_VEL;
                double oldZVel = Z_VEL;
                float rate = (msg->header.stamp.sec - lastSec);
                float rate2 = (msg->header.stamp.nsec - lastNsec);
                rate2 = rate2 * 0.0000000001;
                rate = rate + rate2;
                //std::cout << "rate: " << rate << std::endl;


                double xSign = sign(msg->latitude - ORIGIN_LAT);
                double ySign = sign(msg->longitude - ORIGIN_LON);

                SYSID_THISMAV = msg->system_id;
                
                X_POS = xSign * converter.gps_to_meters(ORIGIN_LAT, ORIGIN_LON, msg->latitude, ORIGIN_LON);
                Y_POS = ySign * converter.gps_to_meters(ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT, msg->longitude);
                Z_POS = msg->altitude;

                X_VEL = (X_POS - oldXPos)/rate;
                Y_VEL = (Y_POS - oldYPos)/rate;
                Z_VEL = (Z_POS - oldZPos)/rate;
                X_ACC = (X_VEL - oldXVel)/rate;
                Y_ACC = (Y_VEL - oldYVel)/rate;
                Z_ACC = (Z_VEL - oldZVel)/rate;

                lastSec = msg->header.stamp.sec;
                lastNsec = msg->header.stamp.nsec;
            }

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
    ros::Publisher pos_publisher = nh.advertise<custom_libs::FullState>("/agent_position_xy", 1, true);

    custom_libs::FullState stateMsg;
    stateMsg.id = thisAgent.SYSID_THISMAV;

    //Publish the xy position
    while( ros::ok() ) {

        stateMsg.position.x = thisAgent.X_POS;
        stateMsg.position.y = thisAgent.Y_POS;
        stateMsg.position.z = thisAgent.Z_POS;
        stateMsg.velocity.x = thisAgent.X_VEL;
        stateMsg.velocity.y = thisAgent.Y_VEL;
        stateMsg.velocity.z = thisAgent.Z_VEL;
        stateMsg.acceleration.x = thisAgent.X_ACC;
        stateMsg.acceleration.y = thisAgent.Y_ACC;
        stateMsg.acceleration.z = thisAgent.Z_ACC;

        stateMsg.header.stamp = ros::Time::now();

        pos_publisher.publish(stateMsg);

        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    return 0;
}