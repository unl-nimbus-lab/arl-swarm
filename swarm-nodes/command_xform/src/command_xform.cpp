#include "command_xform/command_xform.h"

CommandXform::CommandXform() {
    att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 2);
    vector_sub = nh.subscribe("vector", 2, &CommandXform::vectorCallback, this);
}

void CommandXform::vectorCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    float Fx = -1.0 * msg->vector.x;
    float Fy = -1.0 * msg->vector.y;
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
    CommandXform command_xform;
    ros::spin();
    return 0;
}

//OLD version

//What this node does:
//Fx, Fy, Fz -> AttitudeTarget

// mavros_msgs::AttitudeTarget att_target; //attitude target message

// ros::Publisher att_pub; //attitude target publisher
// ros::Subscriber vect_sub; //vector subscriber

// tf2::Quaternion q; //quaternion for attitude target

// float roll_lim = 0.5;
// float pitch_lim = 0.5;

// void vectorCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
//     float Fx = -1.0 * msg->vector.x;
//     float Fy = -1.0 * msg->vector.y;
//     float Fz = msg->vector.z;

//     float roll = atan2(Fy, Fz);
//     // roll = std::max(roll, -roll_lim);
//     // roll = std::min(roll, roll_lim);

//     float pitch = atan2(-Fx, sqrt(Fy*Fy + Fz*Fz));
//     // pitch = std::max(pitch, -pitch_lim);
//     // pitch = std::min(pitch, pitch_lim);

//     float yaw = 0.0;

//     float thrust = sqrt(Fx*Fx + Fy*Fy + Fz*Fz);

//     q.setRPY(roll, pitch, yaw);

//     att_target.orientation.x = q.x();
//     att_target.orientation.y = q.y();
//     att_target.orientation.z = q.z();
//     att_target.orientation.w = q.w();
//     att_target.thrust = thrust;
//     att_target.type_mask = 0b00000111 ; //ignore roll, pitch, yaw rates
//     att_target.header.stamp = ros::Time::now();
//     att_pub.publish(att_target);


// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "command_xform");
//     ros::NodeHandle nh;
    
//     //ros::Publisher command_pub = nh.advertise<std_msgs::String>("command", 1000);
//     att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 2);
//     vect_sub = nh.subscribe("vector", 2, vectorCallback);
    
//     while (ros::ok()) {
//         ros::spin();
//     }
    
//     return 0;
// }