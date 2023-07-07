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
        float ax = 0.0;
        float ay = 0.0;
        float az = 0.0;
        float timeStamp = 0.0;
        std::tuple<hyperRectangle, float> myReachableState;
        std::tuple<hyperRectangle, float> computeRechableSet(
                float positionError, 
                float velocityError, 
                float reachTime, 
                float initialStepSize,
                float reachTimeout) {

            interval posErrorX(-positionError, positionError);
            interval posErrorY(-positionError, positionError);
            interval posErrorZ(z - positionError, z + positionError);
            interval velErrorX(vx - velocityError, vx + velocityError);
            interval velErrorY(vy - velocityError, vy + velocityError);
            interval velErrorZ(vz - velocityError, vz + velocityError);

            std::vector<interval> rectIntervals;

            rectIntervals.push_back(posErrorX);
            rectIntervals.push_back(posErrorY);
            rectIntervals.push_back(posErrorZ);
            rectIntervals.push_back(velErrorX);
            rectIntervals.push_back(velErrorY);
            rectIntervals.push_back(velErrorZ);

            hyperRectangle rect(rectIntervals);
            float acceleration[] = {ax, ay, az};

            std::tuple<hyperRectangle, float> reachableState = verifier.faceLiftingIterativeImprovement(rect, timeStamp, acceleration, reachTime, initialStepSize, reachTimeout);

            return {std::get<0>(reachableState), std::get<1>(reachableState)};
        }
        void state_update_cb( const geometry_msgs::PointStamped::ConstPtr& msg ) {
            x = msg->point.x;
            y = msg->point.y;
            z = msg->point.z;
        }

};


int main(int argc, char **argv) {

    //Initialize ROS
    ros::init(argc, argv, "verifier.cpp");
    ros::NodeHandle nh;

    agent thisAgent;

    //Setup Publishers and subscribers
    ros::Subscriber pos_subscriber = nh.subscribe("/agent_position_xy", 1, &agent::state_update_cb, &thisAgent);
    //ros::Publisher pos_publisher = nh.advertise<geometry_msgs::PointStamped>("/agent_position_xy", 1, true);

    //Publish the xy position
    while( ros::ok() ) {
        //Calculate my reachable space and send it
        


        ros::Duration(0.2).sleep();
        ros::spinOnce();
    }

    return 0;
}