#include <stdio.h>
#include <ros/ros.h>
#include <mavros_msgs/DebugValue.h>
#include <string>
#include <custom_libs/geo_converter.hpp>

//TODO: Un-hardcode these values
int SYSID_THISMAV = 1;
int CLUSTER_ID = 1;

int* msg_tgt_cluster = new int;
int* msg_command = new int;

//TODO: Add (generate aug state function)

void parseCommand(std::string message, int *cluster, int *command) {
    //Process a command of the form:
    //CXXX$XXX Where the "XXX" after the "C" identifies the cluster and
    //the "XXX" after the "$" identifies the command

     std::string delimiter = "$";
     // remove 'C', 'S', or 'A'
     std::string short_message = message.substr(1);
     // now find the '$'
     int split_idx = short_message.find(delimiter);
     // check for error and std::string::npos
     std::string cluster_str = short_message.substr(0, split_idx);
     // remove cluster and delimiter
     short_message.erase(0, split_idx+delimiter.length());
     std::string command_str = short_message;
     // convert the command to an int
     *cluster = atoi(cluster_str.c_str());
     *command = atoi(command_str.c_str());
}


void commandCallback(const mavros_msgs::DebugValue::ConstPtr& msg) {

    parseCommand(msg->name, msg_tgt_cluster, msg_command);

    switch(*msg_command) {
        case 1: //global position target
            ROS_INFO("Received global position target command");
            break;

        case 2://local position target
            ROS_INFO("Received local position target command");
            break;

        default:
            ROS_ERROR("Invalid command received: %d", *msg_command);
                break;
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "hrl_command_interpret");
    ros::NodeHandle nh;

    ros::Subscriber debug_sub = nh.subscribe("/agent_1/mavros/debug_value/debug_vector", 5, commandCallback);
    
    ros::spin();

    return 0;
}

