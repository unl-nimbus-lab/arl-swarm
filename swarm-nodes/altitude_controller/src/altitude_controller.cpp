#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"

//Ros node that publishes a vector3Stamped to the z_target topicf

float z_target = 400.0; //MSL altitude

void stateCallback(const geometry_msgs::Inertia::ConstPtr& msg) {
    float z = msg->position.z;
    float vz = msg->linear_velocity.z;
    float az = msg->linear_acceleration.z;
    
    float Fz = 0.0;
    float Kp = 0.5;
    float Kd = 0.5;
    float Ki = 0.5;
    
    float error = z_target - z;
    float error_dot = 0.0 - vz;
    float error_integral = 0.0;
    
    Fz = Kp*error + Kd*error_dot + Ki*error_integral;
    
    geometry_msgs::Vector3Stamped fz;
    fz.vector.z = Fz;
    fz_pub.publish(fz);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "altitude_controller");
    ros::NodeHandle nh;
    
    ros::Publisher fz_pub = nh.advertise<geometry_msgs::Vector3Stamped>("fz", 250);
    ros::Subscriber state_sub = nh.subscribe("agent_state", 250, stateCallback);
    
    while (ros::ok()) {
        ros::spin();
    }
    
    return 0;
}