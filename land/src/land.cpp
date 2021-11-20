//
// pixhawk_xform.cpp
//
// This node flips the axes on the mocap system
// so the correct positional data can be streamed
// to the Pixhawk via mavros
//
// Created by Adam Plowcha
//

#include <land.h>

int main(int argc, char** argv)
{

    
    //initialize ros
    ros::init(argc, argv, "land");
    ros::NodeHandle nh;

    ROS_INFO("Landing tracker starting...");

    //Setup pubs/subs/clients
    aruco_pose_sub = nh.subscribe("aruco_single/pose", 100, callbackArucoPose);

    //main processing loop
    while (nh.ok())
    {

        ros::spinOnce();

    }

}

void callbackArucoPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    target = *msg;
    
    double z = target.pose.position.z;
    
    ROS_INFO("z pose: %f", z);
}


