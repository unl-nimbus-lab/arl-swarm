#include <iostream>
#include <cmath>
#include <algorithm>

#include "ros/ros.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"

struct vector {
    
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    void disp() {
        std::cout << "X: " << x << ", Y: " << y << ", Z: " << z << std::endl;
    }
};

struct gains {

    double kp = 0.15;
    double ki = 0.0;
    double kd = 0.0;

};

class position_controller {
    public:
        position_controller();

        ros::NodeHandle nh;

        //Velocity targets to be sent to FMU
        ros::Publisher target_velocity_pub;
        ros::Publisher error_pub;

        //Position in the local XY frame is stored here 
        ros::Subscriber state_sub;
        void state_cb(const geometry_msgs::AccelStamped::ConstPtr& msg);

        //Reference will come as a 3d point in space
        ros::Subscriber ref_sub;
        void ref_cb(const geometry_msgs::PointStamped::ConstPtr& msg);

        ros::Subscriber enable_sub;
        void en_cb(const std_msgs::Bool::ConstPtr& msg);

        ros::Timer controller_timer;
        void compute_control( const ros::TimerEvent& event );

        vector current_ref;
        vector current_state;
        vector integral_error;

        gains gxy;
        gains gz;

        

        vector compute_error();
        vector compute_inputs();

        double max_vel = 10.0;
        double min_vel = -10.0;

        bool publish_enable = false;
};

position_controller::position_controller() {
    //Define publishers
    target_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);
    error_pub = nh.advertise<geometry_msgs::PointStamped>("/position_error", 1);

    //Define subscribers
    state_sub = nh.subscribe("/agent_state", 1, &position_controller::state_cb, this);
    ref_sub =   nh.subscribe("/pos_ref", 1 , &position_controller::ref_cb, this);
    enable_sub = nh.subscribe("/pos_en", 1,  &position_controller::en_cb, this);

    //Timer-driven things
    controller_timer = nh.createTimer(ros::Duration(0.04), &position_controller::compute_control, this);

    current_ref.z = 410.0;
    gz.kp = 0.2;

}

void position_controller::en_cb(const std_msgs::Bool::ConstPtr& msg) {
    publish_enable = msg->data;
}

void position_controller::state_cb(const geometry_msgs::AccelStamped::ConstPtr& msg) {
    //Store the current x, y, and z position in the member vars
    current_state.x = msg->accel.linear.x;
    current_state.y = msg->accel.linear.y;
    current_state.z = msg->accel.linear.z;
}

void position_controller::ref_cb(const geometry_msgs::PointStamped::ConstPtr& msg) {
    //Store the current x, y, and z position reference in the member vars
    current_ref.x = msg->point.x;
    current_ref.y = msg->point.y;
    current_ref.z = msg->point.z;
}

vector position_controller::compute_error() {
    //Compute and return the current error based off the current referece and the current state
    vector temp_error;

    temp_error.x = current_ref.x - current_state.x;
    temp_error.y = current_ref.y - current_state.y;
    temp_error.z = current_ref.z - current_state.z;



    return temp_error;
}

vector normalize(vector input_vector, double scale) {
    double x = input_vector.x;
    double y = input_vector.y;
    double z = input_vector.z;

    double magnitude = x*x + y*y;
    magnitude = sqrt(magnitude);

    vector output_vector;

    output_vector.x = scale*x / magnitude;
    output_vector.y = scale*y / magnitude;
    output_vector.z = z;

    return output_vector;
}

void position_controller::compute_control( const ros::TimerEvent& event) {
    //Timer-based control loop

    //Messages that are used
    geometry_msgs::TwistStamped velocity_msg;
    geometry_msgs::PointStamped error_msg;

    //1) Compute the error
    vector vel_command;

    vector error = compute_error();

    //2) Compute the raw velocity command
    vel_command.x = error.x * gxy.kp;
    vel_command.y = error.y * gxy.kp;
    vel_command.z = error.z * gz.kp;

    error_msg.point.x = error.x;
    error_msg.point.y = error.y;
    error_msg.point.z = error.z;

    vel_command.x = std::max(vel_command.x, min_vel);
    vel_command.x = std::min(vel_command.x, max_vel);

    vel_command.y = std::max(vel_command.y, min_vel);
    vel_command.y = std::min(vel_command.y, max_vel);

    vel_command.z = std::max(vel_command.z, min_vel);
    vel_command.z = std::min(vel_command.z, max_vel);
    

    //3) Write the velocity command to the FMU
    velocity_msg.twist.linear.x = vel_command.x;
    velocity_msg.twist.linear.y = vel_command.y;
    velocity_msg.twist.linear.z = vel_command.z;

    if (publish_enable) {

        velocity_msg.header.stamp = ros::Time::now();
        error_msg.header.stamp = ros::Time::now();


        target_velocity_pub.publish(velocity_msg);
        error_pub.publish(error_msg);
    }

    
}

int main( int argc, char** argv) {
    ros::init(argc, argv, "position_controller");
    position_controller pos_control;
    ros::spin();
    return 0;
}