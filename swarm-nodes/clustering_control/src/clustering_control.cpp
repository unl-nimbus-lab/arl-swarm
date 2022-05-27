#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/DebugValue.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "custom_libs/geo_converter.hpp"

#define PI 3.14159265

int mode = 1;
bool initialPositionFlag = true;
bool finalPositionFlag = true;

mavros_msgs::DebugValue modeBroadcast;
mavros_msgs::DebugValue HRLReference;
mavros_msgs::DebugValue agentAcknowledgement;

geo_converter convObject;

//The drone's name:
double system_ID;

//The cluster's name:
double clusterID;

//drone's position in the cluster (1 or 2 or 3 or 4):
double clusterPosition;

//How many drones are in the cluster:
double clusterSize;

double clusterXOffset;
double clusterYOffset;
double clusterZOffset;
double clusterRadius;

double agentAlt;

//World origin (0,0,0)
double homeLat;
double homeLon;
double homeAlt;

//Rally point 1, where the cluster goes to immediattly after takeoff/drop
double clusterRally1X;
double clusterRally1Y;
double rally1Lat;
double rally1Lon;
double rally1Alt;

//Rally point 2, second cluster location
double clusterRally2X;
double clusterRally2Y;
double rally2Lat;
double rally2Lon;
double rally2Alt;

//Dynamic repositioning local reference
double localXTarget;
double localYTarget;
double localZTarget;

//Vehicle's global target
double targetLat;
double targetLon;
double targetAlt;

//Acknowledgement things
bool acknowledgmentFlag = false;
char acknowledgemntID[3];

void cluster2GPS (double xIn, double yIn, double zIn, double *lat, double *lon, double *alt) {
    //Convert a cluster's relative reference in meters to an agent's gps reference in lat,lon,alt
    double xTarget = xIn + clusterXOffset;
    double yTarget = yIn + clusterYOffset;

    *lat = convObject.meters_to_lat(homeLat, homeLon, localYTarget);
    *lon = convObject.meters_to_lon(homeLat, homeLon, localXTarget);
    *alt = zIn + homeAlt;
}

void stateBroadcast_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    //update the state
    modeBroadcast = *msg;
    mode = modeBroadcast.value_int;
    //ROS_INFO("[%i",mode);
}

void HRLReference_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    //Get the new local cluster reference and turn into a global reference for the
    //vehicle to track
    //ROS_WARN("DEBUG_VECTOR_RECEIVED");
    HRLReference = *msg;
    //TODO: if name == 0 -> set home location
    if (stod(HRLReference.name) == clusterID) {
        acknowledgmentFlag = 1; //"I am going to send an acknowledgement"

        //Grab the new cluster reference
        localXTarget = HRLReference.data[0];
        localYTarget = HRLReference.data[1];
        localZTarget = HRLReference.data[2];

        cluster2GPS (localXTarget, localYTarget, localZTarget, &targetLat, &targetLon, &targetAlt);
        agentAcknowledgement.name = system_ID*-1;
        agentAcknowledgement.type = 1;
        agentAcknowledgement.header.stamp = ros::Time::now();
    }
    else if (stod(HRLReference.name) == 0) {
        if (mode == 3) {
            mode = 1;
        }
        else if (mode != 3) {
            mode = 3;
	    ROS_WARN("Backdoor into Live");
        }
    }
}

double calculate_x_offset (double clusterRadius, double clusterSize, double clusterPosition) {
    //Calculate where the agent should be relative to the origin based off its "spot" in the cluster
    double separation = 360 / clusterSize; //Angular distance between agents in a cluster
    double n = (clusterPosition - 1);
    double offset = clusterRadius*cos(PI/180*(separation*n));
    return offset;
}

double calculate_y_offset (double clusterRadius, double clusterSize, double clusterPosition) {
    //Calculate where the agent should be relative to the origin based off its "spot" in the cluster
    double separation = 360 / clusterSize; //Angular distance between agents in a cluster
    double n = (clusterPosition - 1);
    double offset = clusterRadius*sin(PI/180*(separation*n));
    return offset;
}

int main(int argc, char **argv) {

    //load in command line arguments, oh boy
    system_ID       = std::stod(argv[1]);
    clusterID       = std::stod(argv[2]);
    clusterPosition = std::stod(argv[3]);
    clusterSize     = std::stod(argv[4]);
    clusterRadius   = std::stod(argv[5]);
    agentAlt        = std::stod(argv[6]);
    homeLat         = std::stod(argv[7]);
    homeLon         = std::stod(argv[8]);
    homeAlt         = std::stod(argv[9]);
    clusterRally1X  = std::stod(argv[10]);
    clusterRally1Y  = std::stod(argv[11]);
    clusterRally2X  = std::stod(argv[12]);
    clusterRally2Y  = std::stod(argv[13]);

    std::cout << "System ID is: " << system_ID << std::endl;
    std::cout << "Cluster ID is: " << clusterID << std::endl;
    std::cout << "Cluster Position is: " << clusterPosition << std::endl;
    std::cout << "Cluster Size is: " << clusterSize << std::endl;
    std::cout << "Cluster Radius is: " << clusterRadius << std::endl;
    std::cout << "Agent Alt is: " << agentAlt << std::endl;
    std::cout << "Home Lat is: " << homeLat << std::endl;
    std::cout << "Home Lon is: " << homeLon << std::endl;
    std::cout << "Home Alt is: " << homeAlt << std::endl;
    std::cout << "Cluster Rally 1X: " << clusterRally1X << std::endl;
    std::cout << "Cluster Rally 1Y: " << clusterRally1Y << std::endl;
    std::cout << "Cluster Rally 2X: " << clusterRally2X << std::endl;
    std::cout << "Cluster Rally 2Y: " << clusterRally2Y << std::endl;

    //Calculate where the agent should be relative to the origin based off its "spot" in the cluster
    clusterXOffset = calculate_x_offset(clusterRadius, clusterSize, clusterPosition);
    clusterYOffset = calculate_y_offset(clusterRadius, clusterSize, clusterPosition);

    cluster2GPS(clusterRally1X,clusterRally1Y,agentAlt,&rally1Lat,&rally1Lon,&rally1Alt);
    cluster2GPS(clusterRally2X,clusterRally2Y,agentAlt,&rally2Lat,&rally2Lon,&rally2Alt);

    ros::init(argc, argv, "clustering_control");
    ros::NodeHandle nh;

    ros::Subscriber stateBroadcast_sub = nh.subscribe("/mavros/debug_value/named_value_int", 5, stateBroadcast_cb);
    ros::Subscriber liveHRLReference_sub = nh.subscribe("/mavros/debug_value/debug_vector", 5, HRLReference_cb);

    ros::Publisher wp_pub = nh.advertise<geographic_msgs::GeoPoseStamped> ( "/mavros/setpoint_position/global", 5, true );
    ros::Publisher ak_pub = nh.advertise<mavros_msgs::DebugValue> ( "/mavros/debug_value/send", 1, true );

    targetLat = rally1Lat;
    targetLon = rally1Lon;
    targetAlt = rally1Alt;

    while (ros::ok()) {

    geographic_msgs::GeoPoseStamped pos_msg ;

    // State machine as switch 
    switch(mode) {

        case 0 :
            //Rally Point 2
            if (finalPositionFlag) {
                initialPositionFlag = true;
                finalPositionFlag = false;
                ROS_WARN("MOVING TO Rally Point 2 ...");
            }
            pos_msg.pose.position.latitude = rally2Lat;
            pos_msg.pose.position.longitude = rally2Lon;
            pos_msg.pose.position.altitude = rally2Alt;
            break;
        
        case 1 :
            //Starting Position
            if (initialPositionFlag) {
                initialPositionFlag = false;
                finalPositionFlag = true;
                ROS_WARN("MOVING TO Rally Point 1 ...");
            }
            pos_msg.pose.position.latitude = rally1Lat;
            pos_msg.pose.position.longitude = rally1Lon;
            pos_msg.pose.position.altitude = rally1Alt;
            break;
        case 3 : 
            //Live HRL
            //ROS_WARN("LIVE HRL");
            pos_msg.pose.position.latitude = targetLat;
            pos_msg.pose.position.longitude = targetLon;
            pos_msg.pose.position.altitude = targetAlt;
            break;
    }
        pos_msg.header.stamp = ros::Time::now();
        wp_pub.publish( pos_msg );
        if (acknowledgmentFlag == true) {
            acknowledgmentFlag = false;
            ROS_WARN("I need to send an acknowledgement");
            agentAcknowledgement.type = 4;
            agentAcknowledgement.header.stamp = ros::Time::now();
            agentAcknowledgement.value_int = system_ID * -1;
            ak_pub.publish(agentAcknowledgement);
        }
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
return 0;
}
