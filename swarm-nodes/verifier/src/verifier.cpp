#include <iostream>
#include "ros/ros.h"
#include "safetyChecker.h"
#include "geometry_msgs/PointStamped.h"
#include "custom_libs/geo_converter.hpp"
#include "mavros_msgs/SwarmGPS.h"
#include "verifier_comms/hyperRectangle.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/DebugValue.h"
#include "geometry_msgs/AccelStamped.h"
#include "std_msgs/Int32.h"
#include <string>
#include <cstring>
#include <chrono>
#include <fstream>
#include <sstream>

#define START_ROS 1
#define STOP_ROS 2

/* This node will subscribe to the agent's xy position topic and calculate it's reachable space */

double getUnixTime() {
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    
    // Convert the time point to Unix time (seconds since epoch)
    std::chrono::duration<double> unix_time = now.time_since_epoch();
    
    // Convert the duration to a floating-point number
    double unix_time_float = unix_time.count();
    
    return unix_time_float;
}

safetyChecker verifier;

class agent {
    /* Agent class holds all state data (both dynaimic and reachable)*/

    public:

        agent();

        ros::NodeHandle nh;

        int SYSID_THISMAV = 0;
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float vx = 0.0;
        float vy = 0.0;
        float vz = 0.0;
        float ax = 0.0;
        float ay = 0.0;
        float az = 0.0;
        double timeStamp = 0.0;
        bool plotBool = false;
        bool ros_on = false;    
        std::string plotFile;
        std::string othrFile;
        char nextFlightMode = 'L';

        ros::Publisher rect_publisher;
        ros::Subscriber physical_state_subscriber;
        ros::Subscriber cyber_state_subscriber;

        //Verification-related states
        hyperRectangle myReachableState;
        double myReachTime;

        verifier_comms::hyperRectangle rsMessage;

        std::tuple<hyperRectangle, float> computeReachableSet(float positionError, float velocityError, float reachTime, float initialStepSize, float reachTimeout) {

            interval posErrorX(x - positionError, x + positionError);
            interval posErrorY(y - positionError, y + positionError);
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

           // Managing the time proved to be a pain, for now we just calculate the reach time from "0.0"
           // and will just convert it to seconds and add it to a freshly updated time stamp

            auto myRs = verifier.faceLiftingIterativeImprovement(rect, 0.0, acceleration, reachTime, initialStepSize, reachTimeout);

            myReachableState = std::get<0>(myRs); //Set the agent's reach space

            timeStamp = getUnixTime();
            myReachTime = std::get<1>(myRs) / 1000.0 + timeStamp; //Set the agent's reach time

            return {std::get<0>(myRs), std::get<1>(myRs) };
        }

        void packRectangleMessage(hyperRectangle rect, verifier_comms::hyperRectangle &msg) {
            /* Pack a hyperRectangle ROS message with data from hypreRectangle object*/
            
            msg.x.min = rect.intervals[0].min;
            msg.x.max = rect.intervals[0].max;
            msg.y.min = rect.intervals[1].min;
            msg.y.max = rect.intervals[1].max;
            msg.z.min = rect.intervals[2].min;
            msg.z.max = rect.intervals[2].max;
            msg.vx.min = rect.intervals[3].min;
            msg.vx.max = rect.intervals[3].max;
            msg.vy.min = rect.intervals[4].min;
            msg.vy.max = rect.intervals[4].max;
            msg.vz.min = rect.intervals[5].min;
            msg.vz.max = rect.intervals[5].max;
        }

        void state_update_cb( const geometry_msgs::AccelStamped::ConstPtr& msg ) {
            x = msg->accel.linear.x;
            y = msg->accel.linear.y;
            z = msg->accel.linear.z;
            vx= msg->accel.angular.x;
            vy= msg->accel.angular.y;
            vz= msg->accel.angular.z;
        }

        void compute_reach_set_cb( const std_msgs::Int32::ConstPtr& msg ) {


            float compute_time = 1.0 / static_cast<float>(msg->data);
            std::cout << "computing a reach set for " << compute_time << " seconds" << std::endl;
            compute_time = compute_time * 0.4;

            double actualTime = getUnixTime();

            computeReachableSet(1.5, 0.5, 4.0, 4.0, compute_time);

            /*
            Because I often forget compute reachable set arguments:
            1) Postition Error (minimum space size)
            2) Velocity Error (minimum increase)
            3) Reach time (how far ahead to look)
            4) Initial Step Size (how big )
            5) Reach timeout
            */

           actualTime = getUnixTime() - actualTime;

            packRectangleMessage(myReachableState, rsMessage);
            rsMessage.header.stamp = ros::Time::now();

            ros::Time tempTime(actualTime);
            rsMessage.header.stamp = tempTime;
            rsMessage.rs_time = tempTime;

            rect_publisher.publish(rsMessage);


        }

};

agent::agent() {

    physical_state_subscriber = nh.subscribe("/agent_state",   1, &agent::state_update_cb, this);
    cyber_state_subscriber = nh.subscribe(   "/cyber_state",   1, &agent::compute_reach_set_cb,  this);

    rect_publisher = nh.advertise<verifier_comms::hyperRectangle>("/my_reach_state", 1, true);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "verifier");

    agent thisAgent;

    ros::spin();

    return 0;
}
