#ifndef HRL_CONTROL_H
#define HRL_CONTROL_H

#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "mavros_msgs/SetMode.h"

class HRLController {
    public:
        HRLController();

        ros::NodeHandle nh;

        ros::Publisher control_pub;
        
        ros::Subscriber state_sub;
        void stateCallback(const geometry_msgs::AccelStamped::ConstPtr& msg);

        ros::Subscriber aug_state_sub;
        void augStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

        ros::ServiceClient set_mode;

        ros::Timer controller_timer;
        void computeControl( const ros::TimerEvent& event );

        //XY dimension
        Eigen::Matrix<double, 4, 1> x;
        Eigen::Matrix<double, 2, 4> k; 
        Eigen::Matrix<double, 4, 1> aug_state;

        //Z dimension PID controller
        double error_z = 0.0;
        double error_z_integral = 0.0;
        double error_z_derivative = 0.0;
        double error_z_previous = 0.0;
        double raw_output_z = 0.0;

    private:
        double thrust_lim = 0.1;
        double thrust_lim_z = 5.0;

        double kp = 0.8;
        double ki = 0.24;
        double kd = 0.375;

        mavros_msgs::SetMode guided_msg;
        mavros_msgs::SetMode brake_msg;
    
};

#endif