#include "hrl_control/hrl_control.h"


 
HRLController::HRLController() {
    
    control_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vector", 2);
    
    state_sub = nh.subscribe("/agent_state", 2, &HRLController::stateCallback, this);
    aug_state_sub = nh.subscribe("/agent_1/aug_state", 1, &HRLController::augStateCallback, this);

    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    guided_msg.request.custom_mode = "GUIDED";
    brake_msg.request.custom_mode = "BRAKE";

    float controller_period = 0.04;
    controller_timer = nh.createTimer(ros::Duration(0.04), &HRLController::computeControl, this);

}

void HRLController::stateCallback(const geometry_msgs::AccelStamped::ConstPtr& msg) {
    //Update the system's state
    x(0,0) = msg->accel.linear.x;
    x(1,0) = msg->accel.linear.y;
    x(2,0) = msg->accel.angular.x;
    x(3,0) = msg->accel.angular.y;
}

void HRLController::augStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    //Update the augmented state from GCS the set to GUIDED mode
    aug_state(0,0) = msg->point.x;
    aug_state(1,0) = msg->point.y;
    set_mode.call(guided_msg);
}

void HRLController::computeControl( const ros::TimerEvent& event ) {

    //First the altitude PID controller (START HERE)
    // //Compute the control
    // Eigen::MatrixXd u = -k*x;
    // Eigen::MatrixXd u_z = -k_z*x_z;

    // //Publish the control
    // geometry_msgs::Vector3Stamped control_msg;
    // control_msg.vector.x = u(0,0);
    // control_msg.vector.y = u(1,0);
    // control_msg.vector.z = u_z(0,0);
    // control_pub.publish(control_msg);
}

int main() {

    HRLController hrl_control;
    return 0;
}