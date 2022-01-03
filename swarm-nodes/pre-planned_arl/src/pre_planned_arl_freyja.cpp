#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/DebugValue.h"
#include "freyja_msgs/ReferenceState.h"
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
//TODO: Add GPS transform

int mode = 2;
bool trajectoryFollowStart = true;
bool finalPositionHoverStart = true;
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

    ros::init(argc, argv, "prePlanned_freyja");
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
float initialVx = 0.0;
float initialVy = 0.0;
float hoverVelocity = 0.0;
float finalX = freyjaTrajectory[1][1000];
float finalY = freyjaTrajectory[2][1000];
float finalVx = 0.0;
float finalVy = 0.0;
//float heightFixed = -3.0;
float yaw = 0.0;



ros::Subscriber stateBroadcast_sub = nh.subscribe("/mavros/debug_value/named_value_int", 1, stateBroadcast_cb);

ros::Publisher refpublisher = nh.advertise<freyja_msgs::ReferenceState>("/reference_state",10);
while (ros::ok())
{
std::stringstream ss;
freyja_msgs::ReferenceState refstate;


// State machine as switch 
switch(mode) {
    case 0 :
        //On the Ground, no gps lock
        break;
    case 1 :
        //On the Ground, with gps lock
        break;
    case 2 :
        //In the air prior to flight plan execution
        //Hover at the starting point
        refstate.pn = initialY;
        refstate.pe = initialX;
        refstate.pd = heightFixed;
        refstate.vn = hoverVelocity;
        refstate.ve = 0.0;
        refstate.vd = 0.0;
        break;
    case 3 :
        //Set Freyja references from text file
        if (trajectoryFollowStart)
            {
                trajectoryFollowStart = false;
                init_time = ros::Time::now();
                ROS_WARN("Start Trajectory Follow");
            }
        length = ros::Time::now() - init_time;
        sysTime = length.toSec();
        while (freyjaTrajectory[0][columnIndex] < sysTime)
            columnIndex++;
        if (columnIndex < (numberOfColumns -1)) {
            refstate.pn = freyjaTrajectory[2][columnIndex]; //Y position
            refstate.pe = freyjaTrajectory[1][columnIndex]; //X position
            refstate.pd = heightFixed;                      //Fixed height
            refstate.vn = freyjaTrajectory[4][columnIndex]; //Y velocity
            refstate.ve = freyjaTrajectory[3][columnIndex]; //X velocity
            refstate.vd = hoverVelocity;
        }
        else {
        //Hover at the last position
        if (finalPositionHoverStart) {
            finalPositionHoverStart = false;
            ROS_WARN("Hovering at Final Position");
        }
        refstate.pn = finalY;
        refstate.pe = finalX;
        refstate.pd = heightFixed;
        refstate.vn = finalVx;
        refstate.ve = finalVy;
        refstate.vd = hoverVelocity;
        break;
        }
        break;
}
    refstate.header.stamp = ros::Time::now();
    refpublisher.publish( refstate );
    ros::Duration(0.05).sleep();
    ros::spinOnce();
}
return 0;
}