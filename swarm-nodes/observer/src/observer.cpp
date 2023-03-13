#include <stdio.h>
#include "ros/ros.h"
#include "mavros_msgs/SwarmGPS.h"
#include "mavros_msgs/SetMode.h"


int myID = 1;
float myLat = 0.0;
float myLon = 0.0;
float latTol = 0.0005;
float lonTol = 0.0005;

int workingID;
float workingLat;
float workingLon;

float upperLat = 0.0;
float lowerLat = 0.0;

float upperLon = 0.0;
float lowerLon = 0.0;

mavros_msgs::SetMode mode_msg;

char thisAgent[10];
char bogeyAgent[10];

ros::Subscriber global_position_sub;
ros::ServiceClient set_mode;

void GPS_cb(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
    workingID = msg->system_id;
    //If the message is from the host, update the host
    if(workingID == myID) {
        myLat = msg->latitude;
        myLon = msg->longitude;

        upperLat = myLat + latTol;
        lowerLat = myLat - latTol;

        upperLon = myLon + lonTol;
        lowerLon = myLon - lonTol;
    }
    //If the message is not from the host, figure out if it is in the deadzone
    else {
        workingLat = msg->latitude;
        workingLon = msg->longitude;

        if(workingLat < upperLat && workingLat > lowerLat) {
            if (workingLon < upperLon && workingLon > lowerLon) {
                //This vehicle is in the airspace, altert the user
                std::cout << "Agent_" << workingID << " is in my airspace, Calling Brake" << std::endl;
                mode_msg.request.custom_mode = "BRAKE";
                set_mode.call(mode_msg);
            }
        }
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "observer");
    ros::NodeHandle nh;
    global_position_sub = nh.subscribe("/agent_1/mavros/global_position/swarm", 1, GPS_cb);
    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/agent_1/mavros/set_mode");

    ros::spin();

}

