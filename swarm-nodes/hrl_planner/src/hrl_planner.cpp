#include "hrl_planner.h"

//Global Variables
float hrl_publish_rate = 0.5;
int hrl_propogation_steps = 5;

//READ IN TRANSFORM MATRICIES

//READ IN GAIN MATRIX

//STATE UPDATE CALLBACK
void state_update_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    X << msg->point.x, msg->point.y, 0.0, 0.0, 0.0, 0.0;
    //std::cout << "my state is: " << std::endl;
    //ROS_WARN("callback");
    //ROS_INFO_STREAM("X: " << msg->point.x << " Y: " << msg->point.y);
}

//Specify the setpoints
double x_target, y_target;

int main(int argc, char **argv) {

    //Assign Values to Variables:
    ROS_WARN("NODE STARTED");

    x_target = 25.0;
    y_target = 60.0;

    A << 0, 0, 1, 0, 0, 0,
     0, 0, 0, 1, 0, 0,
     0, 0, -1, 0, 0, 0,
     0, 0, 0, -1, 0, 0,
     1, 0, 0, 0, 0, 0,
     0, 1, 0, 0, 0, 0;

    B << 0, 0,
         0, 0,
         -1, 0,
         0, -1,
         0, 0,
         0, 0;

    K << -2.5, 0, -1.6494, 0, -1, 0,
         0, -2.5, 0, -1.6494, 0, -1;

    I << 0, 0, 0, 0, x_target, y_target;

    //Setup the hrl_planner node
    ros::init(argc, argv, "hrl_planner");
    ros::NodeHandle nh;

    //ROS subscribers
    ros::Subscriber swarm_state_sub = nh.subscribe("/agent_position_xy",1, state_update_callback);

    //ROS publishers
    ros::Publisher gps_setpoint_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10, true);

    //begin main ROS LOOP
    while (ros::ok()) {
        ROS_WARN("LOOP");

        T_X = X;

        do {
            //Calculate the control input, u = -kx
            U = -K*T_X;
            ROS_INFO_STREAM("U1: " << U(0,0) << " U2: " << U(1,0));

            //std::cout << "\nStart\n" << A << "\n" << X << "\n" << I << "\n" << B << "\n" << U << std::endl;

            //Propogate the control input forward, x + 1 = Ax + Bu + Zd
            DX = A*T_X - I + B*U;
            ROS_INFO_STREAM("NEXT TARGET: X: " << DX(0,0) << " Y: " << DX(1,0) );
            T_X = DX;
        } while ( (DX(0,0) - X(0,0)) < movement_threshold );

        //publish the next setpoint to the FMU
        //gps_setpoint_pub.publish(setpoint_msg);

        ros::Duration(hrl_publish_rate).sleep();
        ros::spinOnce();
    }

    return 0;
}
