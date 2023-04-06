#include "ros/ros.h"
#include "swarm_comms/allAgentsPosStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/SwarmGPS.h"
#include "custom_libs/geo_converter.hpp"
#include "geometry_msgs/PointStamped.h"


void velocity_update_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void position_update_callback(const mavros_msgs::SwarmGPS::ConstPtr& msg);

//global declared variables
float agent_global_x;
float agent_global_y;
geo_converter convObject;

//global initialized variables
double RMF_ORIGIN_LAT = 40.8467784;
double RMF_ORIGIN_LON = -96.4719180;

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "state_conditioning");
    ros::NodeHandle nh;
    //ros::Subscriber swarm_state_sub = nh.subscribe("/agent_pos", 10, state_update_cb); NEED TO USE THIS ONE IN THE FUTURE
    ros::Subscriber globalPosition_sub = nh.subscribe("/mavros/global_position/swarm",1, position_update_callback);
    ros::Publisher  agent_locating_pub = nh.advertise<geometry_msgs::PointStamped>("/agent_position_xy",1,true);
    
}

void position_update_callback(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
    double latitude = msg->latitude;
    double longitude = msg->longitude;
    
    //Calculate the x position, hold the longitude constant
    agent_global_x = convObject.gps_to_meters(RMF_ORIGIN_LAT,RMF_ORIGIN_LON,latitude,RMF_ORIGIN_LON);
    //Calculate the y position, hold teh latitude constant
    agent_global_y = convObject.gps_to_meters(RMF_ORIGIN_LAT,RMF_ORIGIN_LON,RMF_ORIGIN_LAT,longitude);
}