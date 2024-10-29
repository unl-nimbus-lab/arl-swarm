#ifndef VERIFIER_H
#define VERIFIER_H

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
#include "std_msgs/Float32.h"
#include <string>
#include <cstring>
#include <chrono>
#include <fstream>
#include <sstream>

#include <filesystem>
#include <iomanip>

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

        //Agent's state variables
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

        float reachTime = 4.0;

        //Ploting locations
        std::string plotFile;
        std::string othrFile;

        //Flight mode things
        char nextFlightMode = 'L';

        //ROS publishers and subscribers
        ros::Publisher rect_publisher; //<-- Topic where reachable state gets published
        ros::Subscriber physical_state_subscriber;
        ros::Subscriber reach_time_subscriber;
        ros::Subscriber cyber_state_subscriber; //<< Get rid of this

        //ROS timers
        float reachTimeout = 0.005;
        ros::Timer rs_timer;
        void compute_rs( const ros::TimerEvent& event );


        //Verification-related states
        hyperRectangle myReachableState;
        double myReachTime;

        verifier_comms::hyperRectangle rsMessage;

        //Logging things
        ros::Publisher rs_pub;
        ros::Publisher en_pub;

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

        void reach_timeout_cb( const std_msgs::Float32::ConstPtr& msg ) {
            ROS_WARN("Setting reach timeout to %f", msg->data);
            reachTimeout = msg->data;
        }

        // Get rid of this function
        void compute_reach_set_cb( const std_msgs::Int32::ConstPtr& msg ) {


        //     float compute_time = 1.0 / static_cast<float>(msg->data);
        //     std::cout << "computing a reach set for " << compute_time << " seconds" << std::endl;
        //     compute_time = compute_time * 0.4;

        //     double actualTime = getUnixTime();

        //     computeReachableSet(1.5, 0.5, 4.0, 4.0, compute_time);

        //     /*
        //     Because I often forget compute reachable set arguments:
        //     1) Postition Error (minimum space size)
        //     2) Velocity Error (minimum increase)
        //     3) Reach time (how far ahead to look)
        //     4) Initial Step Size (how big )
        //     5) Reach timeout
        //     */

        //     actualTime = getUnixTime() - actualTime;

        //     packRectangleMessage(myReachableState, rsMessage);
        //     rsMessage.header.stamp = ros::Time::now();

        //     ros::Time tempTime(actualTime);
        //     rsMessage.header.stamp = tempTime;
        //     rsMessage.rs_time = tempTime;

        //     rect_publisher.publish(rsMessage);
        ROS_ERROR("SHOULD NOT SEE THIS MESSAGE");

        }

};

#endif
