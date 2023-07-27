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
    ros::Publisher cmd_pub5 = nh.advertise<geographic_msgs::GeoPoseStamped>("/agent_5/mavros/setpoint_position/global", 1, true);
    ros::Publisher cmd_pub6 = nh.advertise<geographic_msgs::GeoPoseStamped>("/agent_6/mavros/setpoint_position/global", 1, true);
    ros::Publisher cmd_pub7 = nh.advertise<geographic_msgs::GeoPoseStamped>("/agent_7/mavros/setpoint_position/global", 1, true);
    ros::Publisher cmd_pub8 = nh.advertise<geographic_msgs::GeoPoseStamped>("/agent_8/mavros/setpoint_position/global", 1, true);

    geographic_msgs::GeoPoseStamped agent1;
    geographic_msgs::GeoPoseStamped agent2;
    geographic_msgs::GeoPoseStamped agent3;
    geographic_msgs::GeoPoseStamped agent4;
    geographic_msgs::GeoPoseStamped agent5;
    geographic_msgs::GeoPoseStamped agent6;
    geographic_msgs::GeoPoseStamped agent7;
    geographic_msgs::GeoPoseStamped agent8;

    agent1.pose.position.latitude = 40.8474048;
    agent1.pose.position.longitude = -96.4694338;
    agent1.pose.position.altitude = 400;

    agent2.pose.position.latitude = 40.8474048;
    agent2.pose.position.longitude = -96.473261;
    agent2.pose.position.altitude = 400;

    agent3.pose.position.latitude = 40.8446626;
    agent3.pose.position.longitude = -96.473261;
    agent3.pose.position.altitude = 400;

    agent4.pose.position.latitude = 40.8446626;
    agent4.pose.position.longitude = -96.4694338;
    agent4.pose.position.altitude = 400;
 
    agent5.pose.position.latitude = 40.8460337;
    agent5.pose.position.longitude = -96.473261;
    agent5.pose.position.altitude = 400;

    agent6.pose.position.latitude = 40.8460337;
    agent6.pose.position.longitude = -96.4694338;
    agent6.pose.position.altitude = 400;

    agent7.pose.position.latitude = 40.8446626;
    agent7.pose.position.longitude = -96.4713474;
    agent7.pose.position.altitude = 400;

    agent8.pose.position.latitude = 40.8474048;
    agent8.pose.position.longitude = -96.4713474;
    agent8.pose.position.altitude = 400;


    while (ros::ok() ) {

        agent1.header.stamp = ros::Time::now();
        agent2.header.stamp = ros::Time::now();
        agent3.header.stamp = ros::Time::now();
        agent4.header.stamp = ros::Time::now();
        agent5.header.stamp = ros::Time::now();
        agent6.header.stamp = ros::Time::now();
        agent7.header.stamp = ros::Time::now();
        agent8.header.stamp = ros::Time::now();

        cmd_pub1.publish(agent1);
        cmd_pub2.publish(agent2);
        cmd_pub3.publish(agent3);
        cmd_pub4.publish(agent4);
        cmd_pub5.publish(agent5);
        cmd_pub6.publish(agent6);
        cmd_pub7.publish(agent7);
        cmd_pub8.publish(agent8);

        ros::Duration(0.5).sleep();
    }
    return 0;
}