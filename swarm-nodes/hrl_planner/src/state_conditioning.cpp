#include "ros/ros.h"
#include "swarm_comms/allAgentsPosStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/SwarmGPS.h"
#include "custom_libs/geo_converter.hpp"
#include "geometry_msgs/PointStamped.h"


void velocity_update_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void position_update_callback(const mavros_msgs::SwarmGPS::ConstPtr& msg);
double sign(double input);

ros::Publisher agent_location_pub;
ros::Subscriber global_position_sub;

//global declared variables
double agent_global_x;
double agent_global_y;
geo_converter convObject;

//global initialized variables
double RMF_ORIGIN_LAT = 40.8467784;
double RMF_ORIGIN_LON = -96.4719180;
geometry_msgs::PointStamped agent_pos;

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "state_conditioning");
    ros::NodeHandle nh;
    //ros::Subscriber swarm_state_sub = nh.subscribe("/agent_pos", 10, state_update_cb); NEED TO USE THIS ONE IN THE FUTURE
    global_position_sub = nh.subscribe("/mavros/global_position/swarm",1, position_update_callback);
    agent_location_pub = nh.advertise<geometry_msgs::PointStamped>("/agent_position_xy",1,true);

    ros::spin();
    
}

void position_update_callback(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
    double latitude = msg->latitude;
    double longitude = msg->longitude;
    
    //Assumption: we are in the northern hemisphere, this will affect the sign of the local frame
    double x_sign = sign(latitude - RMF_ORIGIN_LAT);
    double y_sign = sign(longitude - RMF_ORIGIN_LON);

    //Calculate the x position, hold the longitude constant
    agent_global_x = convObject.gps_to_meters(RMF_ORIGIN_LAT,RMF_ORIGIN_LON,latitude,RMF_ORIGIN_LON);
    agent_global_x = x_sign * agent_global_x;
    
    //Calculate the y position, hold teh latitude constant
    agent_global_y = convObject.gps_to_meters(RMF_ORIGIN_LAT,RMF_ORIGIN_LON,RMF_ORIGIN_LAT,longitude);
    agent_global_y = y_sign * agent_global_y;

    agent_pos.point.x = agent_global_x;
    agent_pos.point.y = agent_global_y;
    agent_pos.header.stamp = ros::Time::now();

    agent_location_pub.publish(agent_pos);
}

double sign(double input) {
    if (input < 0) {
        return -1.0;
    } else {
        return 1.0;
    }
}