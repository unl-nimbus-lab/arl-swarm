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
#include <stdio.h>
#include <stdlib.h>
#include "custom_libs/geo_converter.hpp"

#define PI 3.14159265
#define NAMED_VALUE_VECT 1
#define NAMED_VALUE_FLOAT 3
#define NAMED_VALUE_INT 4

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
int *override = new int;
ros::NodeHandle nh;
ros::Publisher ak_pub = nh.advertise<mavros_msgs::DebugValue> ( "/mavros/debug_value/send", 10, true );

void parseCommand(std::string message, int *cluster, int *command) {
    //Process a command of the form:
    //CXXX#XXX Where the "XXX" after the "C" identifies the cluster and
    //the "XXX" after the "#" identifies the command
    int length = message.size();
    char tempCluster[10];
    char tempCommand[10];
    int tempIndexCluster = 0;
    int tempIndexCommand = 0;
    for (int i = 0; i < length; i++) {
        //Find the Cluster
        if (message[i] == 'C') {
            for (int j = (i + 1); j < 10; j++) {
                if (message[j] == '#') {
                    i = j - 1;
                    break;
                }
                else {
                    tempCluster[tempIndexCluster] = message[j];
                    tempIndexCluster++;
                }
            }
        }
        //Find the Command
        else if (message[i] == '#') {
            int remainingLength = (length - 1) - i;
            for (int j = i + 1; j < (i+ 1 + remainingLength); j++) {
                tempCommand[tempIndexCommand] = message[j];
                tempIndexCommand++;
            }
        }
    }
    *cluster = atoi(tempCluster);
    *command = atoi(tempCommand);
}

void cluster2GPS (double xIn, double yIn, double zIn, double *lat, double *lon, double *alt) {
    //Convert a cluster's relative reference in meters to an agent's gps reference in lat,lon,alt
    double xTarget = xIn + clusterXOffset;
    double yTarget = yIn + clusterYOffset;

    *lat = convObject.meters_to_lat(homeLat, homeLon, xTarget);
    *lon = convObject.meters_to_lon(homeLat, homeLon, yTarget);
    *alt = zIn + homeAlt;
}

void stateBroadcast_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    //update the state
    modeBroadcast = *msg;
    mode = modeBroadcast.value_int;
    //ROS_INFO("[%i",mode);
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

void acknowledge(int agentID,int* command) {
    //just build the message
    agentAcknowledgement.type = NAMED_VALUE_FLOAT;
    agentAcknowledgement.header.stamp = ros::Time::now();
    
    //build and publish the acknowledgement
    char str[10];
    sprintf(str, "A%d#%d",agentID,*command);
    ak_pub.publish(agentAcknowledgement);
}

void HRLReference_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    HRLReference = *msg;

    int *command = new int;
    int *cluster = new int;

    //Parse the command
    parseCommand(HRLReference.name, cluster, command);

    if (*cluster == 0) { //backdoor case, doesn't matter to the cluster
        //mode switch to dynamic repositioning
        mode = 3;
        //send acknowledgement
        *override = 0;
        acknowledge(system_ID,override);
    }
    else if (*cluster == clusterID) {
        switch (*command) {
            case 1: //mode switch
                //set the mode
                mode = HRLReference.data[0];
                //send acknowledgement
                acknowledge(system_ID,command);
                break;
            case 2: //repositioning location
                //set the reference
                //Grab the new cluster reference
                localXTarget = HRLReference.data[0];
                localYTarget = HRLReference.data[1];
                localZTarget = HRLReference.data[2] + agentAlt;

                //std::cout << "X: " << localXTarget << std::endl;
                //std::cout << "Y: " << localYTarget << std::endl;
                //std::cout << "Z: " << localZTarget << std::endl;
                cluster2GPS (localXTarget, localYTarget, localZTarget, &targetLat, &targetLon, &targetAlt);
                //send acknowledgement
                acknowledge(system_ID,command);
                break;
        }
    }
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

    std::cout << "Cluster X offset: " << clusterXOffset << std::endl;
    std::cout << "Cluster Y offset: " << clusterYOffset << std::endl;

    cluster2GPS(clusterRally1X,clusterRally1Y,agentAlt,&rally1Lat,&rally1Lon,&rally1Alt);
    cluster2GPS(clusterRally2X,clusterRally2Y,agentAlt,&rally2Lat,&rally2Lon,&rally2Alt);

    ros::init(argc, argv, "clustering_control");
    
    //ros::Subscriber stateBroadcast_sub = nh.subscribe("/mavros/debug_value/named_value_int", 10, stateBroadcast_cb);
    ros::Subscriber liveHRLReference_sub = nh.subscribe("/mavros/debug_value/debug_vector", 10, HRLReference_cb);

    ros::Publisher wp_pub = nh.advertise<geographic_msgs::GeoPoseStamped> ( "/mavros/setpoint_position/global", 10, true );
    

    targetLat = rally1Lat;
    targetLon = rally1Lon;
    targetAlt = rally1Alt;

    while (ros::ok()) {

    geographic_msgs::GeoPoseStamped pos_msg ;
 
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
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    }
return 0;
}

//This might be helpful

// # Msg for Debug MAVLink API
// #
// # Supported types:
// # DEBUG			https://mavlink.io/en/messages/common.html#DEBUG
// # DEBUG_VECTOR			https://mavlink.io/en/messages/common.html#DEBUG_VECT
// # NAMED_VALUE_FLOAT		https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT
// # NAMED_VALUE_INT		https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT
// # @TODO: add support for DEBUG_ARRAY (https://github.com/mavlink/mavlink/pull/734)

// std_msgs/Header header

// int32 index			# index value of DEBUG value (-1 if not indexed)

// string name			# value name/key

// float32 value_float		# float value for NAMED_VALUE_FLOAT and DEBUG
// int32 value_int		# int value for NAMED_VALUE_INT
// float32[] data			# DEBUG vector or array

// uint8 type
// uint8 TYPE_DEBUG		= 0
// uint8 TYPE_DEBUG_VECT		= 1
// uint8 TYPE_DEBUG_ARRAY		= 2
// uint8 TYPE_NAMED_VALUE_FLOAT	= 3
// uint8 TYPE_NAMED_VALUE_INT	= 4