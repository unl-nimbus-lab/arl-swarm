//
// pixhawk_xform.h
//
// This node flips the axes on the mocap system
// so the correct positional data can be streamed
// to the Pixhawk via mavros
//
// Created by Adam Plowcha  
//

#ifndef PIXHAWK_XFORM_H_
#define PIXHAWK_XFORM_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

//Publishers and their var
ros::Publisher xform_pub;
geometry_msgs::PoseStamped mocap;
geometry_msgs::PoseStamped xformed;

geometry_msgs::PoseStamped incoming_rel_pos_adjust;

ros::Publisher outgoing_pos_pub;
geometry_msgs::PoseStamped outgoing_rel_pos_adjust;

ros::Publisher status_pub;
std_msgs::String status;

ros::Publisher range_pub;

//Subscribers
ros::Subscriber mocap_sub;
void callbackMocap(const geometry_msgs::PoseStampedConstPtr &msg);

ros::Subscriber command_sub;
void callbackCommand(const std_msgs::StringConstPtr &msg);

ros::Subscriber incoming_pos_adjust_sub;
void callbackIncomingPosAdjust(const geometry_msgs::PoseStampedConstPtr &msg);

//Vars
double angle; // The angle of adjustment to align nose with room frame

//Helper methods
geometry_msgs::Pose getDegFromQuat(geometry_msgs::Pose p);
double rad_to_deg(double x) {return x * (180.0 / 3.14159);}
double deg_to_rad(double x) {return (x * 3.14159) / 180.0;}

#endif // PIXHAWK_XFORM_H_
