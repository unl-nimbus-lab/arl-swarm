#ifndef HRL_CONTROL_H
#define HRL_CONTROL_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "mavros_msgs/SetMode.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "custom_libs/json_reader.hpp"
#include <chrono>

class HRLController {
    public:
        HRLController();

        ros::NodeHandle nh;

        ros::Publisher control_pub;
        ros::Publisher cyber_state_pub;
        
        ros::Subscriber state_sub;
        void stateCallback(const geometry_msgs::AccelStamped::ConstPtr& msg);

        ros::Subscriber aug_state_sub;
        void augStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

        ros::Subscriber publish_state_sub;
        void publishStateCallback(const std_msgs::Bool::ConstPtr& msg);

        ros::ServiceClient set_mode;

        ros::Timer controller_timer;
        void computeControl( const ros::TimerEvent& event );

        Eigen::MatrixXd vectorIdXs(Eigen::MatrixXd input);
        void vectorIdXsTest();
        Eigen::MatrixXd generateAugStateIDXs(Eigen::MatrixXd swarmComp);
        void generateAugStateIDXsTest();
        Eigen::MatrixXd generateIntStateIDXs(Eigen::MatrixXd swarmComp);
        void generateIntStateIDXsTest();
        Eigen::MatrixXd generateRelativeIntstateIDXs(Eigen::MatrixXd intStateIDXs, Eigen::MatrixXd swarmComp);

        double getUnixTime();

        json_reader reader;
        struct Sys system_matricies;
        std::vector<struct dSys> gain_schedule;

        Eigen::Matrix<double, 4, 1> aug_state;

        Eigen::MatrixXd X;
        Eigen::MatrixXd AUG_STATE;
        Eigen::MatrixXd OUTPUT_RAW;
        Eigen::MatrixXd SWARM_COMP;
        Eigen::MatrixXd IDXs;
        Eigen::MatrixXd AUG_STATE_IDXs;
        Eigen::MatrixXd INT_STATE_IDXs;
        Eigen::MatrixXd IT;
        Eigen::MatrixXd REL_INT_STATE_IDXs;
        Eigen::MatrixXd ZD_REL;
        
        Eigen::MatrixXd ERROR;
        Eigen::MatrixXd REL_ERROR;
        Eigen::MatrixXd KCP;
        
        //Z dimension PID controller
        Eigen::Matrix<double, 3, 1> x_z;
        double error_z = 0.0;
        double error_z_integral = 0.0;
        double error_z_derivative = 0.0;
        double error_z_previous = 0.0;
        double raw_output_z = 0.0;
        double target_z = 400.0;
        double control_mod = 1.0;

        //general control things
        double fx = 0.0;
        double fy = 0.0;
        double fz = 0.0;
        double controller_period = 0.04; //seconds
        bool publish_flag = false;
        double thrust_scalar = 1.0;

        geometry_msgs::Vector3Stamped control_msg;
        std_msgs::Int32 cyber_state_msg;

        int SYSID_THISMAV;
        int CLSID_THISMAV;
        
        int OUTIDX;

        int numberOfIntStates = 0;

        int cyber_state = 25; //Hz
        int cyber_reference = 10; //Hz
        float cyber_input = 0.0; //Unitless
        double Kc  = 5.0; //cyber-system gain
        double Kcp = 0.5; //physical system gain

    private:
        double thrust_lim = 0.1;
        double thrust_lim_z = 5.0;
        double thrust_min = 0.15;

        double kp = 0.8;
        double ki = 0.24;
        double kd = 0.375;

        mavros_msgs::SetMode guided_msg;
        mavros_msgs::SetMode brake_msg;
    
};

#endif