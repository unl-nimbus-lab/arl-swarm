#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>



//What this node does:
//Fx, Fy, Fz -> AttitudeTarget

mavros_msgs::AttitudeTarget att_target; //attitude target message

ros::Publisher att_pub; //attitude target publisher
ros::Subscriber vect_sub; //vector subscriber

tf2::Quaternion q; //quaternion for attitude target

void vectorCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    float Fx = msg->vector.x;
    float Fy = msg->vector.y;
    float Fz = msg->vector.z;

    float roll = atan2(Fy, Fz);
    float pitch = atan2(-Fx, sqrt(Fy*Fy + Fz*Fz));
    float yaw = 0.0;
    float thrust = sqrt(Fx*Fx + Fy*Fy + Fz*Fz);

    q.setRPY(roll, pitch, yaw);

    att_target.orientation.x = q.x();
    att_target.orientation.y = q.y();
    att_target.orientation.z = q.z();
    att_target.orientation.w = q.w();
    att_target.thrust = thrust;
    att_target.type_mask = 0b00000111 ; //ignore roll, pitch, yaw rates
    att_target.header.stamp = ros::Time::now();
    att_pub.publish(att_target);


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "command_xform");
    ros::NodeHandle nh;
    
    //ros::Publisher command_pub = nh.advertise<std_msgs::String>("command", 1000);
    att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/agent_1/mavros/setpoint_raw/attitude", 250);
    vect_sub = nh.subscribe("vector", 250, vectorCallback);

    
    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
        ros::spin();
    }
    
    return 0;
}