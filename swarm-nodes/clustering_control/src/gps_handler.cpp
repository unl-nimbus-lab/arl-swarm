#include "ros/ros.h"
#include "mavros_msgs/SwarmGPS.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include <stdio.h>


void gps_cb(const mavros_msgs::SwarmGPS::ConstPtr& msg);
mavros_msgs::SwarmGPS Target;
sensor_msgs::NavSatFix ftlTarget;
float lat, lon, alt;

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_handler");
    ros::NodeHandle nh;

    //Subscribers
    ros::Subscriber swarmState_sub = nh.subscribe("/mavros/global_position/swarm",1, gps_cb);

    //Publishers
    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix> ( "/ftl_target", 1, true );

    while (ros::ok()) {
        ftlTarget.header.stamp = ros::Time::now();
        gps_pub.publish( ftlTarget );
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    return 0;

}

void gps_cb(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
    Target = *msg;
    if (Target.system_id == 4) {
        ftlTarget.latitude = Target.latitude;
        ftlTarget.longitude = Target.longitude;
        ftlTarget.altitude = Target.altitude;
    }

}