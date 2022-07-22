#ifndef CLUSTERING_H
#define CLUSTERING_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/DebugValue.h"
#include "mavros_msgs/SwarmGPS.h"
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

//Universal constants:
#define PI 3.14159265

//MAVROS message types:
#define NAMED_VALUE_VECT 1
#define NAMED_VALUE_FLOAT 3
#define NAMED_VALUE_INT 4

//Some objects that are used:
ros::Publisher ak_pub;
ros::Publisher wp_pub;
mavros_msgs::DebugValue modeBroadcast;
mavros_msgs::DebugValue agentAcknowledgement;
mavros_msgs::DebugValue bootAcknowledgeMsg;
geo_converter convObject;

//Function prototypes:
void parseCommand(std::string message, int *cluster, int *command);
void handleCommand(int *cluster, int *command);
void HRLReference_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
void FTL_cb(const mavros_msgs::SwarmGPS::ConstPtr& msg);
void acknowledge(int agentID,int* command);

//agent class:
class agent {
    public:
        void setSysid(double id);
        void setClusterid(double id);
        void setClusterOrder(double order);
        void setClusterSize(double size);
        void setClusterRadius(double radius);
        void setAgentAlt(double alt);
        void setHome(double lat, double lon, double alt);
        void setRally(double x, double y);
        void setFlightMode(int mode);
        void setClusterOffsets();
        void setPositionTarget();
        void displayCommandlineArguments();
        void setClusterLocalTarget(double x, double y, double z);
        void setClusterType(int type);

        double getSysid();

        double circleXOffset();
        double circleYOffset();

        double calculateXLineOffset();
        double calculateYLineOffset();

        int getFTLTarget();
        int getFlightMode();

        void setFTLTarget(double lat, double lon, double alt);
        void setFTLTargetSystem(int target);
        void setGPSOffsets();
        double latOffset();
        double lonOffset();


        geographic_msgs::GeoPoseStamped pos_msg;
    private:
        double SYSID;
        double CLUSTERID;
        int HRL_FLIGHTMODE = 1;
        double CLUSTER_ORDER;      //AGENT'S CLUSTER POSITION
        double CLUSTER_SIZE;
        int CLUSTER_TYPE = 1;
        double CLUSTER_RADIUS;
        double AGENT_ALT;
        int FTL_TARGET_SYSTEM = 4;
        struct {
            double X;
            double Y;
            double Z;
        } CLUSTER_OFFSETS, AGENT_LOCAL_TARGET, CLUSTER_LOCAL_TARGET;
        struct {
            double LAT;
            double LON;
            double ALT;
        } AGENT_RALLY, CLUSTER_RALLY, GLOBAL_ORIGIN, FTL_TARGET, GPS_OFFSETS;
        
};

void agent::setPositionTarget() {
    //This function is used to update the global target of the agent (pos_msg) based on the hrl flightmode
    switch (HRL_FLIGHTMODE) {
        case 1: //Basic user-defined waypoint navigation
            //Update the Agent Target
            AGENT_LOCAL_TARGET.X = CLUSTER_LOCAL_TARGET.X + CLUSTER_OFFSETS.X;
            AGENT_LOCAL_TARGET.Y = CLUSTER_LOCAL_TARGET.Y + CLUSTER_OFFSETS.Y;
            AGENT_LOCAL_TARGET.Z = CLUSTER_LOCAL_TARGET.Z + CLUSTER_OFFSETS.Z;

            //Update the agent's global position target
            pos_msg.pose.position.latitude = convObject.meters_to_lat(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.X);
            pos_msg.pose.position.longitude = convObject.meters_to_lon(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.Y);
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        case 2: //Follow the Leader
            setGPSOffsets();
            pos_msg.pose.position.latitude = FTL_TARGET.LAT + GPS_OFFSETS.LAT;
            pos_msg.pose.position.longitude = FTL_TARGET.LON + GPS_OFFSETS.LON;
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + GPS_OFFSETS.ALT;
            break;
    };
}

void agent::setFlightMode(int mode) {
    HRL_FLIGHTMODE = mode;
    printf("Flight mode set to %d",mode);
}

void agent::setClusterOffsets() {
    //This function is used to update the cluster offsets, ONLY CALL AFTER THE SPACING HAS BEEN SET!!
    switch (CLUSTER_TYPE) {
        case 1: //Circular cluster formation
            CLUSTER_OFFSETS.X = circleXOffset();
            CLUSTER_OFFSETS.Y = circleYOffset();
            CLUSTER_OFFSETS.Z = AGENT_ALT;
            break;
        case 2: //Line cluster formation
            CLUSTER_OFFSETS.X = calculateXLineOffset();
            CLUSTER_OFFSETS.Y = calculateYLineOffset();
            CLUSTER_OFFSETS.Z = AGENT_ALT;
            break;
    }
}

void agent::setGPSOffsets() {
    //Set the offsets for Follow the Leader
    GPS_OFFSETS.LAT = latOffset();
    GPS_OFFSETS.LON = lonOffset();
    GPS_OFFSETS.ALT = AGENT_ALT;
    printf("GPS offSets Set");
}

double agent::latOffset() {
    //Calculate the cluster offset in terms of Latitude
    double latOffset = convObject.meters_to_lat(FTL_TARGET.LAT,FTL_TARGET.LON,CLUSTER_LOCAL_TARGET.X);
    return latOffset;
}

double agent::lonOffset() {
    //Calculate the cluster offset in terms of Longitude
    double lonOffset = convObject.meters_to_lon(FTL_TARGET.LAT,FTL_TARGET.LON,CLUSTER_LOCAL_TARGET.Y);
    return lonOffset;
}

double agent::circleXOffset() {
    double separation = 360 / CLUSTER_SIZE; //Angular offset between agents in the cluster
    double n = CLUSTER_ORDER - 1;
    double offset = CLUSTER_RADIUS*cos(PI/180*(separation*n));
    return offset;
}

double agent::circleYOffset() {
    double separation = 360 / CLUSTER_SIZE; //Angular offset between agents in the cluster
    double n = CLUSTER_ORDER - 1;
    double offset = CLUSTER_RADIUS*sin(PI/180*(separation*n));
    return offset;
}

void agent::displayCommandlineArguments() {
    //Display command line arguments for debugging
    printf("System ID is: %f \n", SYSID);
    printf("Cluster ID is: %f \n", CLUSTERID);
    printf("Cluster Order is: %f \n", CLUSTER_ORDER);
    printf("Cluster Size is: %f \n", CLUSTER_SIZE);
    printf("Agent Altitude is: %f \n", AGENT_ALT);
    printf("Cluster Radius is: %f \n", CLUSTER_RADIUS);
    printf("Home Lat is: %f \n", GLOBAL_ORIGIN.LAT);
    printf("Home Lon is: %f \n", GLOBAL_ORIGIN.LON);
    printf("Home Alt is: %f \n", GLOBAL_ORIGIN.ALT);
    printf("Cluster Rally X is: %f \n", CLUSTER_RALLY.LAT);
    printf("Cluster Rally Y is: %f \n", CLUSTER_RALLY.LON);
    printf("Cluster Rally Z is: %f \n", CLUSTER_RALLY.ALT);
}

void agent::setSysid(double id) {
    SYSID = id;
}

void agent::setClusterid(double id) {
    CLUSTERID = id;
}

void agent::setClusterOrder(double order) {
    CLUSTER_ORDER = order;
}

void agent::setClusterSize(double size) {
    CLUSTER_SIZE = size;
}

void agent::setClusterRadius(double radius) {
    CLUSTER_RADIUS = radius;
}

void agent::setAgentAlt(double alt) {
    AGENT_ALT = alt;
}

void agent::setHome(double lat, double lon, double alt) {
    GLOBAL_ORIGIN.LAT = lat;
    GLOBAL_ORIGIN.LON = lon;
    GLOBAL_ORIGIN.ALT = alt;
}

void agent::setRally(double x, double y) {
    CLUSTER_RALLY.LAT = x;
    CLUSTER_RALLY.LON = y;
    CLUSTER_RALLY.ALT = AGENT_ALT;
}

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
        if (message[i] == 'C' || message[i] == 'S' || message[i] == 'A') {
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
};

void agent::setClusterLocalTarget(double x, double y, double z) {
    CLUSTER_LOCAL_TARGET.X = x;
    CLUSTER_LOCAL_TARGET.Y = y;
    CLUSTER_LOCAL_TARGET.Z = z;
    printf("Cluster local target set to: %f, %f, %f",x,y,z);
}

void agent::setClusterType(int type) {
    CLUSTER_TYPE = type;
    printf("Cluster changed to type %d\n",type);
}

void acknowledge(int agentID,int* command) {
    agentAcknowledgement.type = NAMED_VALUE_FLOAT;
    agentAcknowledgement.header.stamp = ros::Time::now();
    
    //build and publish the acknowledgement
    char str[10];
    sprintf(str, "A%d#%d", agentID, *command);
    agentAcknowledgement.name = str; 
    ak_pub.publish(agentAcknowledgement);
}

double agent::getSysid() {
    return SYSID;
}

double agent::calculateXLineOffset() {
    double length = (CLUSTER_SIZE - 1) * CLUSTER_RADIUS;
    double offset = (CLUSTER_ORDER - 1) * length;
    return offset;
}

double agent::calculateYLineOffset() {
    return 0;
}

int agent::getFTLTarget() {
    return FTL_TARGET_SYSTEM;
}

int agent::getFlightMode() {
    return HRL_FLIGHTMODE;
}

void agent::setFTLTarget(double lat, double lon, double alt) {
    FTL_TARGET.LAT = lat;
    FTL_TARGET.LON = lon;
    FTL_TARGET.ALT = alt;
}

void agent::setFTLTargetSystem(int target) {
    FTL_TARGET_SYSTEM = target;
    printf("FTL Target set to %d\n",target);
}


#endif