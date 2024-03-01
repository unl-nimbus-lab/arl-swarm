#include "hrl_control/hrl_control.h"
 
HRLController::HRLController() {
    //Initialize the ROS node publishers, subscribers, and services
    control_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vector", 2);
    
    state_sub = nh.subscribe("/agent_state", 2, &HRLController::stateCallback, this);
    aug_state_sub = nh.subscribe("/aug_state", 1, &HRLController::augStateCallback, this);
    publish_state_sub = nh.subscribe("/publish_state", 1, &HRLController::publishStateCallback, this);

    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    guided_msg.request.custom_mode = "GUIDED";
    brake_msg.request.custom_mode = "BRAKE";

    //Timer event that tiggers the main control loop
    controller_timer = nh.createTimer(ros::Duration(0.04), &HRLController::computeControl, this);

    //Initialize the state and gain matricies
    x << 0.0, 0.0, 0.0, 0.0;
    k << 0.9879, 0.0000, 1.814, 0.0000,
        0.0000, 0.9879, 0.0000, 1.8140;

    x_z << 0.0, 0.0, 0.0;

    aug_state << 30.0, -10.0, 0.0, 0.0;
}

void HRLController::stateCallback(const geometry_msgs::AccelStamped::ConstPtr& msg) {
    //Update the system's state from the message
    x(0,0) = msg->accel.linear.x;
    x(1,0) = msg->accel.linear.y;
    x(2,0) = msg->accel.angular.x;
    x(3,0) = msg->accel.angular.y;

    x_z(0,0) = msg->accel.linear.z;
    x_z(1,0) = msg->accel.angular.z;
}

void HRLController::augStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    //Update the augmented state from GCS the set to GUIDED mode
    aug_state(0,0) = msg->point.x;
    aug_state(1,0) = msg->point.y;
    set_mode.call(guided_msg);
}

void HRLController::publishStateCallback(const std_msgs::Bool::ConstPtr& msg) {
    //Update the flag to publish the state
    publish_flag = msg->data;
}

void HRLController::computeControl( const ros::TimerEvent& event ) {

    if (publish_flag) {

        //First the altitude PID controller
        error_z = target_z - x_z(0,0); //compute the error

        //If the altitude error is too large, only adjust the altitude
        if ( abs( error_z ) > 3.0 ) {
            control_mod  = 0.0; //turn off the XY controller
            raw_output_z = kp*error_z;//Only going to use P control to avoid large windup
            error_z_integral = 0.0;
            ROS_WARN("Altitude error too large, Adjusting height only");
        } else {
            control_mod = 1.0;
            //Compute the PID control signal for the altitude
            error_z_integral += error_z * controller_period;
            error_z_derivative = (error_z - error_z_previous) / controller_period;
            error_z_previous = error_z;
            raw_output_z = kp*error_z + ki*error_z_integral + kd*error_z_derivative;
        }

        //Now the XY controller, LQR control
        output_raw = -1.0 * k * (x - aug_state);
        auto error = x - aug_state;

        //Limit the control signal to the thrust limits
        fx = std::max( output_raw(0,0), -1 * thrust_lim);
        fx = std::min( fx, thrust_lim);

        fy = std::max( output_raw(1,0), -1 * thrust_lim);
        fy = std::min( fy, thrust_lim);

        fz = std::max( raw_output_z, thrust_min );//Note, should always have a non-zero thrust for Z
        fz = std::min( fz, thrust_lim_z);
        
        control_msg.vector.x = control_mod*fx;
        control_msg.vector.y = control_mod*fy;
        control_msg.vector.z = fz;

        //To avoid dancing around the target, switch to BRAKE mode if close, uploading a new
        //waypoint will automatically switch back to GUIDED mode when it is recieved. 
        if (error.norm() < 5.0) {
            ROS_WARN("Close to target, stopping");
            set_mode.call(brake_msg);
        }

        control_pub.publish(control_msg);

        //update new controller rate
        //float controller_period = 1.0/controller_rate_;
        //controller_timer_.setPeriod( ros::Duration(controller_period), true);
    }

}

int main( int argc, char** argv ) {

    ros::init(argc, argv, "hrl_control");
    HRLController hrl_control;
    ros::spin();
    return 0;
}