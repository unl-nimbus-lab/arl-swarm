#ifndef HRL_PLANNER_H
#define HRL_PLANNER_H

#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>

//ROS message includes
#include "geographic_msgs/GeoPoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "swarm_comms/allAgentsPosStamped.h"


Eigen::MatrixXd A(6,6), B(6,2), K(2,6), X(6,1), T_X(6,1), U(2,1), I(6,1), DX(6,1);

double movement_threshold = 3.0;

#endif