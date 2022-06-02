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

mavros_msgs::DebugValue acknowledgement_msg;

int *agent = new int;
int *command = new int;

void parseCommand(std::string message, int *cluster, int *command) {

    int length = message.size();
    char tempCluster[10];
    char tempCommand[10];
    int tempIndexCluster = 0;
    int tempIndexCommand = 0;
    //Find the Cluster
    for (int i = 0; i < length; i++) {
        if (message[i] == 'A') {
            for (int j = (i + 1); j < 10; j++) {
                if (message[j] == '#') {
                    i = j - 1;
                    break;
                }
                else {
                    tempCluster[tempIndexCluster] = message[j];
                    tempIndexCluster++;
                }
            }
        }
        else if (message[i] == '#') {
            int remainingLength = (length - 1) - i;
            for (int j = i + 1; j < (i+ 1 + remainingLength); j++) {
                tempCommand[tempIndexCommand] = message[j];
                tempIndexCommand++;
            }
        }
    }
    *agent = atoi(tempCluster);
    *command = atoi(tempCommand);
}

void acknowledgment_cb (const mavros_msgs::DebugValue::ConstPtr& msg) {
    acknowledgement_msg = *msg;
    parseCommand(acknowledgement_msg.name,agent,command);
    printf("Agent %d acknowleged command %d\n",*agent,*command);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ack");
    ros::NodeHandle nh;

    ros::Subscriber acknowledgement_sub = nh.subscribe("/mavros/debug_value/named_value_float", 100, acknowledgment_cb);

    ros::spin();
}