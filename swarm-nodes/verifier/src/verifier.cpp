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

/* This node will subscribe to the agent's xy position topic and calculate it's reachable space */

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
        float timeStamp = 0.0;
        bool plotBool = false;    
        std::string plotFile;
        std::string othrFile;


        //These json objects are for plotting this agent's rs's
        nlohmann::json j;
        int jsonTrack = 1;
        
        //These json objects are for plotting recieved agent's rs's
        nlohmann::json aj;
        int ajsonTrack = 1;

        std::tuple<hyperRectangle, float> myReachableState;

        std::tuple<hyperRectangle, float> computeRechableSet(
                float positionError, 
                float velocityError, 
                float reachTime, 
                float initialStepSize,
                float reachTimeout) {

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

            myReachableState = verifier.faceLiftingIterativeImprovement(rect, timeStamp, acceleration, reachTime, initialStepSize, reachTimeout);
            //std::get<0>(myReachableState).toJson(j,jsonTrack);

            return {std::get<0>(myReachableState), std::get<1>(myReachableState)};
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

        void intersection_cb( const verifier_comms::hyperRectangle::ConstPtr& msg ) {
            int inId = msg->id;
            if (inId != SYSID_THISMAV) {

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

                if (plotBool == true) {
                    receivedState.toJson(aj,ajsonTrack);
                    aj[0]["Rectangles"] = ajsonTrack - 1;
                    std::ofstream outputFile(othrFile);
                    if(outputFile.is_open() ) {
                        outputFile << aj.dump(4);
                        outputFile.close();
                        std::cout << "JSONFILE SAVED" << std::endl;
                    }
                }

                auto intersects = std::get<0>(myReachableState).intersects(receivedState);

                if (intersects) {
                    std::cout << "Detected Collision with agent: " << inId << std::endl;
                    mavros_msgs::SetMode mode_msg;
                    mode_msg.request.custom_mode = "BRAKE";
                    set_mode.call(mode_msg);     
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

        agent(int id) {
            /* Constructor to set the system ID for the Agent*/
            SYSID_THISMAV = id;
        }
};


int main(int argc, char **argv) {

    agent thisAgent(atoi(argv[1]));

    for (int i =  0; i < argc; i++ ) {
        std::cout << "arg " << i << " is " << argv[i] << std::endl;
    }

    // if (argc > 6){
    //     std::string plotFile = argv[2];
    //     plotBool = true;
    //     std::cout << "Plotting Enabled" << std::endl;
    //     //std::cout << "saving to " << fullFile << std::endl;
    // }

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

    //Setup Publishers and subscribers
    ros::Subscriber pos_subscriber = nh.subscribe("/agent_position_xy", 1, &agent::state_update_cb, &thisAgent);
    ros::Subscriber rs_subscriber = nh.subscribe("/received_state", 10, &agent::intersection_cb, &thisAgent);

    ros::Publisher pos_publisher = nh.advertise<verifier_comms::hyperRectangle>("/my_reach_state", 1, true);

    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    //Time Things
    ros::Time time_begin = ros::Time::now();
    ros::Time time_end = ros::Time::now();
    ros::Duration duration = time_end - time_begin;

    verifier_comms::hyperRectangle rsMessage;
    rsMessage.id = thisAgent.SYSID_THISMAV;

    while( ros::ok() ) {
    //     //Calculate my reachable space and send it

        duration = ros::Time::now() - time_begin;
        float reachTime = duration.toSec() * 1000;
        thisAgent.timeStamp = reachTime;

        /*
        Because I forget compute reachable set arguments:
        1) Postition Error
        2) Velocity Error
        3) Reach time
        4) Initial Step Size
        5) Reach timeout
        */

        thisAgent.computeRechableSet(4.0 ,0.5 ,3.0, 0.05, 0.005);

        std::get<0>(thisAgent.myReachableState).toJson(thisAgent.j,thisAgent.jsonTrack);

        if (thisAgent.plotBool == true) {
            thisAgent.j[0]["Rectangles"] = thisAgent.jsonTrack - 1;
            std::ofstream outputFile(thisAgent.plotFile);
            if(outputFile.is_open() ) {
                outputFile << thisAgent.j.dump(4);
                outputFile.close();
                std::cout << "JSONFILE SAVED" << std::endl;
            }
        }

        thisAgent.packRectangleMessage(std::get<0>(thisAgent.myReachableState), rsMessage);


        rsMessage.header.stamp = ros::Time::now();

        pos_publisher.publish(rsMessage);


        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    return 0;
}
