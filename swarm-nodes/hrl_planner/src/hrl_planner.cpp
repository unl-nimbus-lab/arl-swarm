#include "hrl_planner.h"

//Global Variables
float hrl_publish_rate = 0.5;
int hrl_propogation_steps = 5;

//READ IN TRANSFORM MATRICIES

//READ IN GAIN MATRIX

//STATE UPDATE CALLBACK

//Specify the setpoints
double x_target, y_target;


int main(int argc, char **argv) {

    //Assign Values to Variables:

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
    //ros::Subscriber swarm_state_sub = nh.subscribe("") (Need the topic for the state sub)

    //ROS publishers
    ros::Publisher gps_setpoint_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10, true);

    //begin main ROS LOOP
    while (ros::ok()) {

        for (int i = 0; i < hrl_propogation_steps; i++) {
            //Calculate the control input, u = -kx

            //Propogate the control input forward, x + 1 = Ax + Bu + Zd
        }

        //publish the next setpoint to the FMU
        //gps_setpoint_pub.publish(setpoint_msg);

        ros::Duration(hrl_publish_rate).sleep();
        ros::spinOnce();
    }

    return 0;
}


