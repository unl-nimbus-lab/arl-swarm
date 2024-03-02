#include <stdio.h>
#include <ros/ros.h>
#include <mavros_msgs/DebugValue.h>
#include <string>
#include <custom_libs/geo_converter.hpp>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>

geo_converter converter;

struct { //Global origin is stored here, should not change
    float lat = 40.846740;
    float lon = -96.471819;
} global_origin;

//TODO: Un-hardcode these values
int SYSID_THISMAV = 1;
int CLUSTER_ID = 1;

int* msg_tgt_cluster = new int;
int* msg_command = new int;

geometry_msgs::PointStamped aug_state;

ros::Publisher aug_state_pub;
ros::Publisher publish_state_pub;

double sign(double input) {
    //Deterine the sign of a double
    if (input < 0) {
        return -1.0;
    } else {
        return 1.0;
    }
}

double lat = 0.0;
double lon = 0.0;
double x_target = 0.0;
double y_target = 0.0;
double x_sign = 0.0;
double y_sign = 0.0;

void updateAugState(float x_target, float y_target) {
    aug_state.header.stamp = ros::Time::now();
    aug_state.point.x = x_target;
    aug_state.point.y = y_target;
    aug_state.point.z = 0.0;
    aug_state_pub.publish(aug_state);
}

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
        case 1: //Command type 1: tagets as global position
            lat = msg->data[0];
            lon = msg->data[1];

            //Assumption: we are working in the northern hemisphere
            y_sign = sign(lat - global_origin.lat);
            x_sign = sign(lon - global_origin.lon);

            y_target = converter.gps_to_meters(global_origin.lat, global_origin.lon, lat, global_origin.lon);
            y_target = y_sign * y_target;

            x_target = converter.gps_to_meters(global_origin.lat, global_origin.lon, global_origin.lat, lon);
            x_target = x_sign * x_target;
            //conert to local
            updateAugState(x_target, y_target);
                break;

        case 2://Command type 2: target as local position
            ROS_INFO("Received local position target command");
            updateAugState(msg->data[0], msg->data[1]);
                break;

        default:
            ROS_ERROR("Invalid command received: %d", *msg_command);
                break;
    }

}

void debugIntCallback(const mavros_msgs::DebugValue::ConstPtr& msg) {
    std_msgs::Bool publish_state;
    switch(msg->value_int) {
        case 1:
            publish_state.data = true;
            publish_state_pub.publish(publish_state);
                break;
        case 2:
            publish_state.data = false;
            publish_state_pub.publish(publish_state);
                break;
        default:
            ROS_ERROR("Invalid INT command received");
                break;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "hrl_command_interpret");
    ros::NodeHandle nh;

    aug_state_pub = nh.advertise<geometry_msgs::PointStamped>("/aug_state", 2);
    publish_state_pub = nh.advertise<std_msgs::Bool>("/publish_state", 2);

    ros::Subscriber debug_sub = nh.subscribe("/mavros/debug_value/debug_vector", 5, commandCallback);
    ros::Subscriber debug_int_sub = nh.subscribe("/mavros/debug_value/named_value_int", 5, debugIntCallback);
    
    ros::spin();

    return 0;
}

