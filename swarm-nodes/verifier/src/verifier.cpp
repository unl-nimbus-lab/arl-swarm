#include <iostream>
#include "ros/ros.h"
#include "safetyChecker.h"
#include "geometry_msgs/PointStamped.h"
#include "custom_libs/FullState.h"
#include "custom_libs/geo_converter.hpp"
#include "mavros_msgs/SwarmGPS.h"
#include "verifier_comms/hyperRectangle.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/DebugValue.h"
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
ros::ServiceClient set_mode;

class agent {
    /* Agent class holds all state data (both dynaimic and reachable)*/

    public:
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

        //These json objects are for plotting this agent's rs's
        nlohmann::json j;
        int jsonTrack = 1;
        
        //These json objects are for plotting recieved agent's rs's
        nlohmann::json aj;
        int ajsonTrack = 1;

        //Verification-related states
        hyperRectangle myReachableState;
        double myReachTime;

        std::ofstream rs_outputFile;
        std::ofstream en_outputFile;
        std::ofstream dc_outputFile;
        std::ofstream br_outputFile;
        std::ofstream in_outputFile;
        std::ofstream hr_outputFile;
        std::ofstream rr_outputFile;

        // //outputfiles
        // std::string logDir = "/home/verifier_logs";
        // std::ofstream rs_outputFile("/home/rs_time.txt");
        // std::ofstream en_outputFile("/home/en_time.txt");
        // std::ofstream dc_outputFile("/home/dc_time.txt");
        // std::ofstream br_outputFile("/home/br_time.txt");
        // std::ofstream in_outputFile("/home/in_time.txt");

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

            /*
            Face lifting Interave improvement arguments
            rect: Initial State of the ego agent
            start_time: global clock
            acceleration: derived from pos_xform
            reachTime: maximum reach time
            initialStepSize: starting step size
            reachTimeout: how long this can run 
            */

           // Managing the time proved to be a pain, for now we just calculate the reach time from "0.0"
           // and will just convert it to seconds and add it to a freshly updated time stamp

            auto myRs = verifier.faceLiftingIterativeImprovement(rect, 0.0, acceleration, reachTime, initialStepSize, reachTimeout);

            myReachableState = std::get<0>(myRs); //Set the agent's reach space

            timeStamp = getUnixTime();
            myReachTime = std::get<1>(myRs) / 1000.0 + timeStamp; //Set the agent's reach time

            return {std::get<0>(myRs), std::get<1>(myRs) };
        }

        void state_update_cb( const custom_libs::FullState::ConstPtr& msg ) {
            x = msg->position.x;
            y = msg->position.y;
            z = msg->position.z;
            vx = msg->velocity.x;
            vy = msg->velocity.y;
            vz = msg->velocity.z;
            ax = msg->acceleration.x;
            ay = msg->acceleration.y;
            az = msg->acceleration.z;
        }

        void cmd_cb( const mavros_msgs::DebugValue::ConstPtr& msg ) {
            int cmd = msg->value_int;
            
            switch(cmd) {
                case START_ROS:
                    printf("STARTING_ROS");
                        ros_on = true;
                    break;
                case STOP_ROS:
                    printf("STOPPING_ROS");
                        ros_on = false;
                    break;
                default:
                    printf("INVALID CMD ID\n");
                    break;

            }
        }

        void intersection_cb( const verifier_comms::hyperRectangle::ConstPtr& msg ) {
            int inId = msg->id;
            if (inId != SYSID_THISMAV) {

                auto br_time = getUnixTime(); //Time broadcast is recieved
                ros::Time broadcastTime = msg->header.stamp;
                double br_start = broadcastTime.toSec(); //Time broadcast was sent

                if (br_outputFile.is_open()) { //Save br time computation to file
                    br_outputFile << std::to_string(br_time - br_start) << std::endl;
                }


                auto dc_start = getUnixTime(); //start decode computation

                ros::Time inTime_ros = msg->rs_time; //The reach time of the space
                double inTime = inTime_ros.toSec();

                auto isUseful = isStateValid(inTime, dc_start);

                if (isUseful) {
                    std::cout << "The Recieved State is valid" << std::endl;
                } else {
                    std::cout << "The Recieved State in too old" << std::endl;
                }

                //Start Decode
                interval x(msg->x.min, msg->x.max);
                interval y(msg->y.min, msg->y.max);
                interval z(msg->z.min, msg->z.max);
                interval vx(msg->vx.min,msg->vx.max);
                interval vy(msg->vy.min,msg->vy.max);
                interval vz(msg->vz.min,msg->vz.max);

                std::vector<interval> rectIntervals;

                rectIntervals.push_back(x);
                rectIntervals.push_back(y);
                rectIntervals.push_back(z);
                rectIntervals.push_back(vx);
                rectIntervals.push_back(vy);
                rectIntervals.push_back(vz);

                hyperRectangle receivedState(rectIntervals);

                auto dc_time = getUnixTime(); //End decode time

                if (dc_outputFile.is_open()) { //Save rs time computation to file
                    dc_outputFile << std::to_string(dc_time - dc_start) << std::endl;
                }

                if (rr_outputFile.is_open() ) {
                    std::stringstream plotString;
                    plotString << "a" << inId << ":" 
                                    << receivedState.intervals[0].min << ":" 
                                    << receivedState.intervals[0].max << ":" 
                                    << receivedState.intervals[1].min << ":" 
                                    << receivedState.intervals[1].max << ":" 
                                    << receivedState.intervals[2].min << ":" 
                                    << receivedState.intervals[2].max;

                    rr_outputFile << plotString.str() << std::endl;
                }

                //Start Intersection
                auto in_start = getUnixTime();
                auto intersects = myReachableState.intersects(receivedState);
                auto in_time = getUnixTime();

                if (in_outputFile.is_open()) { //Save rs time computation to file
                    in_outputFile << std::to_string(in_time - in_start) << std::endl;
                }

                //Stop intersection

                if (intersects) {
                    std::cout << "Detected Collision with agent: " << inId << std::endl;
                    if (ros_on) {
                        mavros_msgs::SetMode mode_msg;
                        mode_msg.request.custom_mode = "BRAKE";
                        set_mode.call(mode_msg);  
                    }
                }
            }
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

        bool isStateValid(double inTime, double myTime) {
            if (inTime >= myTime) {
                return true;
            } else {
                return false;
            }
        }

        agent(int id, std::string logDir) {
            /* Constructor to set the system ID for the Agent*/
            SYSID_THISMAV = id;
            rs_outputFile.open(logDir + "/rs_time.txt");
            en_outputFile.open(logDir + "/en_time.txt");
            dc_outputFile.open(logDir + "/dc_time.txt");
            br_outputFile.open(logDir + "/br_time.txt");
            in_outputFile.open(logDir + "/in_time.txt");
            hr_outputFile.open(logDir + "/reachable_spaces.txt");
            rr_outputFile.open(logDir + "/rr_spaces.txt");
        }
};

int main(int argc, char **argv) {

    agent thisAgent(atoi(argv[1]), argv[3] );

    for (int i =  0; i < argc; i++ ) {
        std::cout << "arg " << i << " is " << argv[i] << std::endl;
    }
    // if (argc > 6){
    //     std::string plotFile = argv[2];
    //     plotBool = true;
    //     std::cout << "Plotting Enabled" << std::endl;
    //     //std::cout << "saving to " << fullFile << std::endl;
    // }

    // This block of code is a janky way of recording intervals for plotting hyperrectangels
    for (int i = 0; i < argc; i++) {
	std::string tempString = argv[i];
        if (tempString == "plotting") {
            thisAgent.plotFile = "/home/jsonlogs/myrs.json";
            thisAgent.othrFile = "/home/jsonlogs/adrs.json";
            thisAgent.plotBool = true;
            std::cout << "Plotting Enabled, ";
            std::cout << "saving to " << thisAgent.plotFile << "and, " << thisAgent.othrFile << std::endl;
        }
    }

    thisAgent.j[0]["Rectangles"] = 0;

    //Initialize ROS
    ros::init(argc, argv, "verifier.cpp");
    ros::NodeHandle nh;

    //Set up publishers, subscribers, and ros services
    ros::Subscriber pos_subscriber = nh.subscribe("/agent_position_xy", 1, &agent::state_update_cb, &thisAgent);
    ros::Subscriber cmd_subscriber = nh.subscribe("/mavros/debug_value/named_value_int", 10, &agent::cmd_cb, &thisAgent);
    ros::Subscriber rs_subscriber = nh.subscribe("/received_state", 10, &agent::intersection_cb, &thisAgent);
    ros::Publisher pos_publisher = nh.advertise<verifier_comms::hyperRectangle>("/my_reach_state", 1, true);

    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    verifier_comms::hyperRectangle rsMessage;
    rsMessage.id = thisAgent.SYSID_THISMAV;

    while( ros::ok() ) {
        //Calculate my reachable space and send it

        auto rs_start = getUnixTime(); //Start Reach Time computation

        /*
        Because I often forget compute reachable set arguments:
        1) Postition Error (minimum space size)
        2) Velocity Error (minimum increase)
        3) Reach time (how far ahead to look)
        4) Initial Step Size (how big )
        5) Reach timeout
        */

        thisAgent.computeReachableSet(1.0 ,0.25 ,3.0, 0.05, 0.005);
        
        auto rs_time = getUnixTime(); //End of Reach Time computation

        //Log the time to calculate reachable space
        if (thisAgent.rs_outputFile.is_open()) { //Save rs time computation to file
            thisAgent.rs_outputFile << std::to_string(rs_time - rs_start) << std::endl;
        }
  

        if (thisAgent.plotBool == true) {
            if (thisAgent.hr_outputFile.is_open() ) {
                std::stringstream plotString;
                plotString << "a" << thisAgent.SYSID_THISMAV << ":" 
                                 << thisAgent.myReachableState.intervals[0].min << ":" 
                                 << thisAgent.myReachableState.intervals[0].max << ":" 
                                 << thisAgent.myReachableState.intervals[1].min << ":" 
                                 << thisAgent.myReachableState.intervals[1].max << ":" 
                                 << thisAgent.myReachableState.intervals[2].min << ":" 
                                 << thisAgent.myReachableState.intervals[2].max;

                thisAgent.hr_outputFile << plotString.str() << std::endl;
            }
        }

        auto en_start = getUnixTime(); //Start encode time computation

        thisAgent.packRectangleMessage(thisAgent.myReachableState, rsMessage);
        rsMessage.header.stamp = ros::Time::now();

        ros::Time tempTime(thisAgent.myReachTime);
        rsMessage.rs_time = tempTime;

        auto en_time = getUnixTime(); //End endcode time computation

        if (thisAgent.en_outputFile.is_open()) { //Save the encode time to file
            thisAgent.en_outputFile << std::to_string(en_time - en_start) << std::endl;
        }

        pos_publisher.publish(rsMessage);
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    return 0;
}
