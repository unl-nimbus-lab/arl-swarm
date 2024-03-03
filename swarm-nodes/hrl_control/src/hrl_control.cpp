#include "hrl_control/hrl_control.h"

//This node uses state feedback to compute control inputs, its behavior can be modified by 
//sending debug vectors (see hrl_command_interpret for more information)
 
HRLController::HRLController() {
    //ROS things
    control_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vector", 2);
    
    state_sub = nh.subscribe("/agent_state", 2, &HRLController::stateCallback, this);
    aug_state_sub = nh.subscribe("/aug_state", 1, &HRLController::augStateCallback, this);
    publish_state_sub = nh.subscribe("/publish_state", 1, &HRLController::publishStateCallback, this);

    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    guided_msg.request.custom_mode = "GUIDED";
    brake_msg.request.custom_mode = "BRAKE";

    //Timer event that tiggers the main control loop
    controller_timer = nh.createTimer(ros::Duration(0.04), &HRLController::computeControl, this);

    //Be sure to use absolute paths here
    system_matricies = reader.readJSON("/home/catkin_ws/src/hrl_control/src/HRL_Sys_2_2.json");
    
    //Resize and initialize the current state of the swarm to zeros since we may not have full feedback yet
    int numberOfHRLStates = system_matricies.ZD.size();
    int numberOfAgents = system_matricies.p.sum();
    ROS_WARN("Number of agents is %d", numberOfAgents);

    nh.param<int>("SystemID", SYSID_THISMAV, 3);
    ROS_WARN("Set SYSID_THISMAV to %d", SYSID_THISMAV);

    IDXs.resize(numberOfAgents,1);
    int count = 0;
    for (int i = 0; i < system_matricies.state_idxs_C.rows(); i++) {
        for (int j = 0; j < system_matricies.state_idxs_C.cols(); j++) {
            IDXs(count,0) = system_matricies.state_idxs_C(i,j);
            count++;
        }
    }

    X.resize(numberOfHRLStates, 1);
    X.setZero();

    //Need a better way to manage the aug_state
    AUG_STATE.resize(numberOfHRLStates, 1);
    AUG_STATE.setZero();
    AUG_STATE(4,0)  = -100.0;
    AUG_STATE(5,0)  =  100.0;
    AUG_STATE(16,0) =  100.0;
    AUG_STATE(17,0) =  100.0;

    //This should be number of agents x 2
    OUTPUT_RAW.resize(4, 1);
    OUTPUT_RAW.setZero();

    IT = system_matricies.T.inverse();

    OUTIDX = (SYSID_THISMAV - 1) * 2;

    // std::cout << "The length of aug_state is " << AUG_STATE.size() << std::endl;
    // std::cout << "AUG_STATE: " << std::endl << AUG_STATE << std::endl;

    
    //Initialize the state and gain matricies
    x << 0.0, 0.0, 0.0, 0.0;
    k << 0.9879, 0.0000, 1.814, 0.0000,
        0.0000, 0.9879, 0.0000, 1.8140;

    x_z << 0.0, 0.0, 0.0;

    aug_state << 30.0, -10.0, 0.0, 0.0;
}

void HRLController::stateCallback(const geometry_msgs::AccelStamped::ConstPtr& msg) {
    //Update the system's state from the message
    // x(0,0) = msg->accel.linear.x;
    // x(1,0) = msg->accel.linear.y;
    // x(2,0) = msg->accel.angular.x;
    // x(3,0) = msg->accel.angular.y;

    x_z(0,0) = msg->accel.linear.z;
    x_z(1,0) = msg->accel.angular.z;

    int i = IDXs(SYSID_THISMAV - 1,0); //Get the idx of this agent


    X = IT*X; //Un-transform X

    X(i,0) = msg->accel.linear.x;
    X(i+1,0) = msg->accel.linear.y;
    X(i+2,0) = msg->accel.angular.x;
    X(i+3,0) = msg->accel.angular.y;
    X = system_matricies.T*X; //Transform X

}

void HRLController::augStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    //Update the augmented state from GCS the set to GUIDED mode
    aug_state(0,0) = msg->point.x;
    aug_state(1,0) = msg->point.y;    std::cout << "IDXs: " << std::endl << IDXs << std::endl;  
    set_mode.call(guided_msg);
}

void HRLController::publishStateCallback(const std_msgs::Bool::ConstPtr& msg) {
    //Update the flag to publish the state
    publish_flag = msg->data;
}


void HRLController::computeControl( const ros::TimerEvent& event ) {

    if (publish_flag) {//Only publish the state if the flag is set

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


        for (int i = 8; i < 12; i++) {
            X(i,0) = std::max( X(i,0), -5.0);
            X(i,0) = std::min( X(i,0), 5.0);
        }

        for (int i = 20; i < 24; i++) {
            X(i,0) = std::max( X(i,0), -5.0);
            X(i,0) = std::min( X(i,0),  5.0);
        }

        OUTPUT_RAW = -1.0 * system_matricies.DKL * (X - AUG_STATE);
        //auto ERROR = X - AUG_STATE;

        OUTPUT_RAW( OUTIDX, 0) = std::max( OUTPUT_RAW( OUTIDX, 0), -1 * thrust_lim);
        OUTPUT_RAW( OUTIDX, 0) = std::min( OUTPUT_RAW( OUTIDX, 0),      thrust_lim);

        OUTPUT_RAW( OUTIDX + 1, 0) = std::max( OUTPUT_RAW( OUTIDX + 1, 0), -1 * thrust_lim);
        OUTPUT_RAW( OUTIDX + 1, 0) = std::min( OUTPUT_RAW( OUTIDX + 1, 0),      thrust_lim);

        std::cout << "OUTPUT_RAW: " << std::endl << OUTPUT_RAW << std::endl;

        fx = OUTPUT_RAW(OUTIDX,     0);
        fy = OUTPUT_RAW(OUTIDX + 1, 0);

        //Now the XY controller, LQR control
        output_raw = -1.0 * k * (x - aug_state);
        auto error = x - aug_state;

        //Limit the control signal to the thrust limits
        // fx = std::max( output_raw(0,0), -1 * thrust_lim);
        // fx = std::min( fx, thrust_lim);

        // fy = std::max( output_raw(1,0), -1 * thrust_lim);
        // fy = std::min( fy, thrust_lim);

        fz = std::max( raw_output_z, thrust_min );//Note, should always have a non-zero thrust for Z
        fz = std::min( fz, thrust_lim_z);
        
        control_msg.vector.x = control_mod*fx;
        control_msg.vector.y = control_mod*fy;
        control_msg.vector.z = fz;

        //To avoid dancing around the target, switch to BRAKE mode if close, uploading a new
        //waypoint will automatically switch back to GUIDED mode when it is recieved. 
        //if (ERROR.norm() < 5.0) {
        //    ROS_WARN("Close to target, stopping");
        //    set_mode.call(brake_msg);
        //}

        control_pub.publish(control_msg);

        X = system_matricies.DTA * (X - AUG_STATE) + system_matricies.DTB * OUTPUT_RAW - system_matricies.ZD * 0.04 + AUG_STATE;
        


        auto X_T = IT*X;
        //std::cout << "X: " << std::endl << X_T << std::endl; 

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