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

char ID[3];

int xTarget;
int yTarget;
int zTarget;
char buffer[3];

mavros_msgs::DebugValue acknowledgement_msg;

void acknowledgment_cb (const mavros_msgs::DebugValue::ConstPtr& msg) {
    ROS_WARN("Got a Debug Message");
    acknowledgement_msg = *msg;
    int id = acknowledgement_msg.value_int;
    if (id < 0) {
        std::cout << "Agent " << id*-1 << " acknowledges new reference\n";
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "live_gcs");
    ros::NodeHandle nh;

    ros::Publisher wp_pub = nh.advertise<mavros_msgs::DebugValue> ( "/mavros/debug_value/send", 5, true );
    
    ros::Subscriber acknowledgement_sub = nh.subscribe("/mavros/debug_value/named_value_int", 5, acknowledgment_cb);

    while (ros::ok()) {
	
	mavros_msgs::DebugValue HRLReference;

        std::cout << "Enter the agents ID: ";
        std::cin >> ID;
        std::cout << "Enter the new X: ";
        std::cin >> xTarget;
        std::cout << "Enter the new Y: ";
        std::cin >> yTarget;
        std::cout << "Enter the new Z: ";
        std::cin >> zTarget;

        //fprintf("You have told Agent %s to got to %d, %d, %d",ID,xTarget,yTarget,zTarget);

        HRLReference.name = ID;
        HRLReference.data.push_back(xTarget);
        HRLReference.data.push_back(yTarget);
        HRLReference.data.push_back(zTarget);
        HRLReference.type = 1;
        HRLReference.header.stamp = ros::Time::now();
        wp_pub.publish( HRLReference );
	ros::spinOnce();
    }
}
