//
// land.h
//
//  
//
// Created by Adam Plowcha  
//

#ifndef LAND_H_
#define LAND_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

//Publishers and their var


//Subscribers
ros::Subscriber aruco_pose_sub;
void callbackArucoPose(const geometry_msgs::PoseStampedConstPtr &msg);
geometry_msgs::PoseStamped target;

//Vars
double angle; // The angle of adjustment to align nose with room frame
bool tracking;

//Helper methods
geometry_msgs::Pose getDegFromQuat(geometry_msgs::Pose p);
double rad_to_deg(double x) {return x * (180.0 / 3.14159);}
double deg_to_rad(double x) {return (x * 3.14159) / 180.0;}

#endif // LAND_H_
