#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/DebugValue.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
//TODO: Add GPS transform

int mode = 1;
bool initialPositionFlag = true;
bool finalPositionFlag = true;
ros::Time init_time;
ros::Duration length;
float sysTime;
int columnIndex = 0;
int numberOfColumns = 1001;
int numberOfRows = 5;
mavros_msgs::DebugValue modeBroadcast;
std::string file_var;
float heightFixed = -3.0;

void stateBroadcast_cb(const mavros_msgs::DebugValue::ConstPtr& msg)
{
    modeBroadcast = *msg;
    mode = modeBroadcast.value_int;
    //ROS_INFO("[%i",mode);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "prePlanned_cube");
    ros::NodeHandle nh;

    if (nh.getParam("/trajectory_file", file_var))
        std::cout << file_var << "\n";
    else
        ROS_ERROR("BAD FILE PARAMETER");

    if (nh.getParam("/height", heightFixed))
        ROS_WARN("HEIGHT REFERNCE IS: %f", heightFixed);
        //std::cout << heightFixed << "\n";
    else
        ROS_ERROR("BAD HEIGHT PARAMETER");

    ////////// BEGIN FILE READ //////////

    using namespace std;
    string line;
    ifstream myfile(file_var);
    float freyjaTrajectory[numberOfRows][numberOfColumns];
    int i = 0;
    int j = 0;
    string temp;

    if (myfile.is_open()) //error handling
    {
        while (myfile && getline(myfile, line)) //split each line from the file into "line"
        {
            stringstream ss(line); // substream takes a string and begins to read it like ifstream does to a file. Creates a stream that we can use
            j = 0;
            while (ss && ss >> temp) { // goes as long as the stream from ss has stuff to read in
                freyjaTrajectory[i][j] = stof(temp);
                j++;
            }
            i++;
        }
        myfile.close();
        ROS_WARN("Trajectory successfuly loaded");
    }

    else 
    {
    ROS_ERROR("Unable to load trajectory");
    return 1;
    }
    ////////// END FILE READ //////////

//Set initial conditions 
float initialX = freyjaTrajectory[1][0];
float initialY = freyjaTrajectory[2][0];
float finalX = freyjaTrajectory[1][1000];
float finalY = freyjaTrajectory[2][1000];
float yaw = 0.0;



ros::Subscriber stateBroadcast_sub = nh.subscribe("/mavros/debug_value/named_value_int", 1, stateBroadcast_cb);

ros::Publisher wp_pub = nh.advertise<geometry_msgs::PoseStamped> ( "/mavros/setpoint_position/local", 1, true );
while (ros::ok())
{

geometry_msgs::PoseStamped pos_msg ;

// State machine as switch 
switch(mode) {

    case 0 :
        //Final ARL Positoin
        if (finalPositionFlag) {
            initialPositionFlag = true;
            finalPositionFlag = false;
            ROS_WARN("MOVING TO FINAL POSITION ...");
        }
        pos_msg.pose.position.x = finalX;
        pos_msg.pose.position.y = finalY;
	    pos_msg.pose.position.z = heightFixed;
        break;
    
    case 1 :
        //Starting Position
        if (initialPositionFlag) {
            initialPositionFlag = false;
            finalPositionFlag = true;
            ROS_WARN("MOVING TO INITIAL POSITION ...");
        }
        pos_msg.pose.position.x = initialX;
        pos_msg.pose.position.y = initialY;
	    pos_msg.pose.position.z = heightFixed;
        break;
}
    pos_msg.header.stamp = ros::Time::now();
    wp_pub.publish( pos_msg );
    ros::Duration(0.5).sleep();
    ros::spinOnce();
}
return 0;
}