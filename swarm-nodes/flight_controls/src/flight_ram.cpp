#include "ros/ros.h"
#include "geographic_msgs/GeoPoseStamped.h"

//Simple node for moving all the UAVs to starting locations

int main(int argc, char** argv) {

    ros::init(argc, argv, "start");
    ros::NodeHandle nh;

    ros::Publisher cmd_pub1 = nh.advertise<geographic_msgs::GeoPoseStamped>("/agent_1/mavros/setpoint_position/global", 1, true);
    ros::Publisher cmd_pub2 = nh.advertise<geographic_msgs::GeoPoseStamped>("/agent_2/mavros/setpoint_position/global", 1, true);
    ros::Publisher cmd_pub3 = nh.advertise<geographic_msgs::GeoPoseStamped>("/agent_3/mavros/setpoint_position/global", 1, true);
    ros::Publisher cmd_pub4 = nh.advertise<geographic_msgs::GeoPoseStamped>("/agent_4/mavros/setpoint_position/global", 1, true);

    geographic_msgs::GeoPoseStamped agent1;
    geographic_msgs::GeoPoseStamped agent2;
    geographic_msgs::GeoPoseStamped agent3;
    geographic_msgs::GeoPoseStamped agent4;

    agent1.pose.position.latitude = 40.8459841;
    agent1.pose.position.longitude = -96.4710332;
    agent1.pose.position.altitude = 400;

    agent2.pose.position.latitude = 40.8459841;
    agent2.pose.position.longitude = -96.4710332;
    agent2.pose.position.altitude = 400;

    agent3.pose.position.latitude = 40.8459841;
    agent3.pose.position.longitude = -96.4710332;
    agent3.pose.position.altitude = 400;

    agent4.pose.position.latitude = 40.8459841;
    agent4.pose.position.longitude = -96.4710332;
    agent4.pose.position.altitude = 400;


    while (ros::ok() ) {

        agent1.header.stamp = ros::Time::now();
        agent2.header.stamp = ros::Time::now();
        agent3.header.stamp = ros::Time::now();
        agent4.header.stamp = ros::Time::now();

        cmd_pub1.publish(agent1);
        cmd_pub2.publish(agent2);
        cmd_pub3.publish(agent3);
        cmd_pub4.publish(agent4);

        ros::Duration(0.5).sleep();
    }
    return 0;
}