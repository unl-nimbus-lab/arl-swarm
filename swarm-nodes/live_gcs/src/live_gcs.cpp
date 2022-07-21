#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/DebugValue.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

int ID;
int command;
int xTarget;
int yTarget;
int zTarget;
char buffer[3];
char name[10];

#define NAMED_VECTOR 1

mavros_msgs::DebugValue acknowledgement_msg;

int main(int argc, char **argv) {

    ros::init(argc, argv, "live_gcs");
    ros::NodeHandle nh;

    ros::Publisher wp_pub = nh.advertise<mavros_msgs::DebugValue> ( "/mavros/debug_value/send", 5, true );

    while (ros::ok()) {
	
	mavros_msgs::DebugValue HRLReference;

    std::cout << "Enter Cluster ID: ";
    std::cin >> ID;
    std::cout << "Enter Command #";
    std::cin >> command;
    std::cout << "Payload 0 (X): ";
    std::cin >> xTarget;
    std::cout << "Payload 1 (Y): ";
    std::cin >> yTarget;
    std::cout << "Payload 2 (Z): ";
    std::cin >> zTarget;

    sprintf(name, "C%d#%d",ID,command);

    HRLReference.name = name;
    HRLReference.data.push_back(xTarget);
    HRLReference.data.push_back(yTarget);
    HRLReference.data.push_back(zTarget);
    HRLReference.type = NAMED_VECTOR;
    HRLReference.header.stamp = ros::Time::now();
    wp_pub.publish( HRLReference );
	ros::spinOnce();
    }
}
