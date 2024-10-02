#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"

struct vector {
    
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    void disp() {
        std::cout << "X: " << x << ", Y: " << y << ", Z: " << z << std::endl;
    }
};

struct gains {

    double kp = 1.0;
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

        ros::Timer controller_timer;
        void compute_control( const ros::TimerEvent& event );

        vector current_ref;
        vector current_state;
        vector integral_error;

        gains gxy;
        gains gz;

        vector compute_error();
        vector compute_inputs();
};

position_controller::position_controller() {
    //Define publishers
    target_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/agent_1/mavros/setpoint_velocity/cmd_vel",1);
    error_pub = nh.advertise<geometry_msgs::PointStamped>("/agent_1/position_error", 1);

    //Define subscribers
    state_sub = nh.subscribe("/agent_1/agent_state", 1, &position_controller::state_cb, this);
    ref_sub =   nh.subscribe("/agent_1/vel_ref", 1 , &position_controller::ref_cb, this);

    //Timer-driven things
    controller_timer = nh.createTimer(ros::Duration(0.04), &position_controller::compute_control, this);

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
    temp_error.z = current_ref.z = current_state.z;

    return temp_error;
}

void position_controller::compute_control( const ros::TimerEvent& event) {
    //Timer-based control loop

    //Messages that are used
    geometry_msgs::TwistStamped velocity_msg;
    geometry_msgs::PointStamped error_msg;
    //1) Compute the error
    vector error;
    vector vel_command;

    error = compute_error();

    //2) Compute the velocity command
    vel_command.x = error.x * gxy.kp;
    vel_command.y = error.y * gxy.kp;
    vel_command.z = 0.0;

    error_msg.point.x = error.x;
    error_msg.point.y = error.y;
    error_msg.point.z = error.z;

    //3) Write the velocity command to the FMU
    velocity_msg.twist.linear.x = vel_command.x;
    velocity_msg.twist.linear.y = vel_command.y;
    velocity_msg.twist.linear.z = vel_command.z;

    target_velocity_pub.publish(velocity_msg);
    error_pub.publish(error_msg);
}

int main( int argc, char** argv) {
    ros::init(argc, argv, "position_controller");
    position_controller pos_control;
    ros::spin();
    return 0;
}