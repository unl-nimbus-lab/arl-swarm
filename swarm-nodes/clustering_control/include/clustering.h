#ifndef CLUSTERING_H
#define CLUSTERING_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/DebugValue.h"
#include "mavros_msgs/SwarmGPS.h"
#include "mavros_msgs/CommandLong.h"
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

//HRL Flightmodes
#define CLUSTER_GUIDED_LOCAL_MODE 1
#define CLUSTER_RALLY_MODE 2
#define CLUSTER_GUIDED_GLOBAL_MODE 3

//Cluster Formation types
#define CIRCLE_FORMATION 1
#define LINEAR_FORMATION 2

//Cluster Commands
#define PING 0
#define START_CLUSTER_GUIDED 1
#define STOP_CLUSTER_GUIDED 2
#define CHANGE_FLIGHT_MODE 3
#define SET_CLUSTER_LOCAL_POSITION 4
#define SET_CLUSTER_RADIUS 5
#define SET_CLUSTER_TYPE 6
#define SET_HOME_POSITION 7
#define SET_LOCAL_RALLY 8
#define SET_CLUSTER_GLOBAL_POSITION 9
#define SEPARATE_ALTITUDES 10

//Agent Commands

#define CHANGE_CLUSTER_ID 2
#define CHANGE_SYSID 3
#define CHANGE_CLUSTER_ORDER 4
#define CHANGE_CLUSTER_SIZE 5
#define CHANGE_AGENT_ALTITUDE 6

//Some objects that are used:
ros::Publisher ak_pub;
ros::Publisher wp_pub;
ros::ServiceClient client;
mavros_msgs::DebugValue modeBroadcast;
mavros_msgs::DebugValue agentAcknowledgement;
mavros_msgs::DebugValue bootAcknowledgeMsg;
mavros_msgs::CommandLong alt_sep;
geo_converter convObject;

//Function prototypes:
void parseCommand(std::string message, int *cluster, int *command);
void handleCommand(int *cluster, int *command);
void HRLReference_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
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
        void setRally1(double x, double y);
        void setRally2(double x, double y);
        double getRally1X();
        double getRally1Y();
        double getRally2X();
        double getRally2Y();
        double getAgentAlt();
        void setFlightMode(int mode);
        void setClusterOffsets();
        void setPositionTarget();
        void setClusterLocalTarget(double x, double y, double z);
        void setClusterType(int type);
        double getSysid();
        double getClusterid();
        double circleXOffset();
        double circleYOffset();
        double calculateXLineOffset();
        double calculateYLineOffset();
        int getFlightMode();
        double latOffset();
        double lonOffset();
        void setAgentLocalTarget(double x, double y, double z);
        void setPublish(bool logic);
        bool getPublish();
        void setClusterGlobalTarget(double lat, double lon, double alt);
        geographic_msgs::GeoPoseStamped pos_msg;

    private:
        double SYSID;               //Unique ID of this agent within the swarm
        double CLUSTERID;           //ID of the cluster this agent belongs to 
        int HRL_FLIGHTMODE = 1;     //HRL "flight mode"
        double CLUSTER_ORDER;       //This agent's "spot" in the cluster
        double CLUSTER_SIZE;        //How many agents are in the cluster
        int CLUSTER_TYPE = 1;       //Organization of agents within the cluster 1) Circle 2) Line
        double CLUSTER_RADIUS;      //Dimension of the cluster
        double AGENT_ALT;           //Height (in meters) above "GLOBAL ORIGIN"
        bool PUBLISH = false;       //O-k to begin publishing messages to the FCU
        struct {
            double X;
            double Y;
            double Z;
        } CLUSTER_OFFSETS, AGENT_LOCAL_TARGET, CLUSTER_LOCAL_TARGET, CLUSTER_RALLY1, CLUSTER_RALLY2;
        //Cluster Offsets: How far (in meters this agent should be offset from the cluster center)
        //Agent Local Target: Local Agent target after offsets applied
        //Cluster Local Target: Location (in meters) the center of the cluster should be relative to "GLOBAL ORIGIN"
        struct {
            double LAT;
            double LON;
            double ALT;
        } AGENT_RALLY, GLOBAL_ORIGIN, GPS_OFFSETS, CLUSTER_GLOBAL_TARGET;
        //Agent Rally: Not used
        //Cluster Rally: Locally saved point, unique to each cluster
        //Global Origin: Common world frame for the swarm
        //CLUSTER_GLOBAL_TARGET: Target lat/lon/alt for the cluster
};

void agent::setPositionTarget() {
    //This function is used to update the global target of the agent (pos_msg) 
    //based on the hrl flightmode, this is the only function that should
    //directly manipulate the pos_msg.
    double tempx, tempy, tempz;

    switch (HRL_FLIGHTMODE) {
        case CLUSTER_GUIDED_LOCAL_MODE: //Move the cluster in the local frame
            //Update "where the agent should be in the local frame"
            tempx = CLUSTER_LOCAL_TARGET.X + CLUSTER_OFFSETS.X;
            tempy = CLUSTER_LOCAL_TARGET.Y + CLUSTER_OFFSETS.Y;
            tempz = CLUSTER_LOCAL_TARGET.Z + CLUSTER_OFFSETS.Z;
            setAgentLocalTarget(tempx,tempy,tempz);

            //Update the pos_msg based on agent local target and global origin
            pos_msg.pose.position.latitude = convObject.meters_to_lat(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.X);
            pos_msg.pose.position.longitude = convObject.meters_to_lon(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.Y);
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        case CLUSTER_RALLY_MODE: //Move the cluster in the local frame
            //Update "where the agent should be in the local frame"
            tempx = CLUSTER_RALLY1.X + CLUSTER_OFFSETS.X;
            tempy = CLUSTER_RALLY1.Y + CLUSTER_OFFSETS.Y;
            tempz = CLUSTER_RALLY1.Z + CLUSTER_OFFSETS.Z;
            setAgentLocalTarget(tempx,tempy,tempz);

            //Update the pos_msg based on agent local target and global origin
            pos_msg.pose.position.latitude = convObject.meters_to_lat(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.X);
            pos_msg.pose.position.longitude = convObject.meters_to_lon(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.Y);
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        case CLUSTER_GUIDED_GLOBAL_MODE:
            //Update "where the agent should be in the global frame"
            pos_msg.pose.position.latitude = convObject.meters_to_lat(CLUSTER_GLOBAL_TARGET.LAT, CLUSTER_GLOBAL_TARGET.LON, CLUSTER_OFFSETS.X);
            pos_msg.pose.position.longitude = convObject.meters_to_lon(CLUSTER_GLOBAL_TARGET.LAT, CLUSTER_GLOBAL_TARGET.LON, CLUSTER_OFFSETS.Y);
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + CLUSTER_GLOBAL_TARGET.ALT;
            break;
        default:
            printf("INVALID FLIGHT MODE REQUESTED\n");
            break;
    };
}

void agent::setAgentLocalTarget(double x, double y, double z) {

    AGENT_LOCAL_TARGET.X = x;
    AGENT_LOCAL_TARGET.Y = y;
    AGENT_LOCAL_TARGET.Z = z;
    printf("Agent Local Target Set: %f, %f, %f\n", AGENT_LOCAL_TARGET.X, AGENT_LOCAL_TARGET.Y, AGENT_LOCAL_TARGET.Z);
}

void agent::setClusterGlobalTarget(double lat, double lon, double alt) {
    CLUSTER_GLOBAL_TARGET.LAT = lat;
    CLUSTER_GLOBAL_TARGET.LON = lon;
    CLUSTER_GLOBAL_TARGET.ALT = alt;
    printf("Agent Global Target Set: %f, %f, %f\n", CLUSTER_GLOBAL_TARGET.LAT, CLUSTER_GLOBAL_TARGET.LON, CLUSTER_GLOBAL_TARGET.ALT);
}

void agent::setFlightMode(int mode) {
    HRL_FLIGHTMODE = mode;
    printf("Flight mode set to %d\n",HRL_FLIGHTMODE);
}

void agent::setClusterOffsets() {
    //This function is used to update the cluster offsets, ONLY CALL AFTER THE RADIUS HAS BEEN SET!!
    switch (CLUSTER_TYPE) {
        case CIRCLE_FORMATION:
            CLUSTER_OFFSETS.X = circleXOffset();
            CLUSTER_OFFSETS.Y = circleYOffset();
            CLUSTER_OFFSETS.Z = AGENT_ALT;
            break;
        case LINEAR_FORMATION: //Line cluster formation
            CLUSTER_OFFSETS.X = calculateXLineOffset();
            CLUSTER_OFFSETS.Y = calculateYLineOffset();
            CLUSTER_OFFSETS.Z = AGENT_ALT;
            break;
        default:
            printf("INVALID CLUSTER TYPE ATTEMPTED\n");
        break;
    }
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

void agent::setSysid(double id) {
    SYSID = id;
    printf("SYSID Set to %f\n",SYSID);
}

void agent::setClusterid(double id) {
    CLUSTERID = id;
    printf("Cluster ID Set to %f\n",CLUSTERID);
}

void agent::setClusterOrder(double order) {
    CLUSTER_ORDER = order;
    printf("Cluster Order Set to %f\n",CLUSTER_ORDER);
}

void agent::setClusterSize(double size) {
    CLUSTER_SIZE = size;
    printf("Cluster size Set to %f\n",CLUSTER_SIZE);
}

void agent::setClusterRadius(double radius) {
    CLUSTER_RADIUS = radius;
    printf("Cluster radius Set to %f\n",CLUSTER_RADIUS);
}

void agent::setAgentAlt(double alt) {
    AGENT_ALT = alt;
    printf("Agent Altitude Set to %f\n",AGENT_ALT);
}

void agent::setHome(double lat, double lon, double alt) {
    GLOBAL_ORIGIN.LAT = lat;
    GLOBAL_ORIGIN.LON = lon;
    GLOBAL_ORIGIN.ALT = alt;
    printf("Global Origin Set to %f, %f, %f\n",GLOBAL_ORIGIN.LAT, GLOBAL_ORIGIN.LON, GLOBAL_ORIGIN.ALT);
}

void agent::setRally1(double x, double y) {
    CLUSTER_RALLY1.X = x;
    CLUSTER_RALLY1.Y = y;
    CLUSTER_RALLY1.Z = AGENT_ALT;
    printf("Cluster Rally 1 Set to %f, %f, %f\n",CLUSTER_RALLY1.X, CLUSTER_RALLY1.Y, CLUSTER_RALLY1.Z);
}

double agent::getRally1X() {
    return CLUSTER_RALLY1.X;
}

double agent::getRally1Y() {
    return CLUSTER_RALLY1.Y;
}

void agent::setRally2(double x, double y) {
    CLUSTER_RALLY2.X = x;
    CLUSTER_RALLY2.Y = y;
    CLUSTER_RALLY2.Z = AGENT_ALT;
    printf("Cluster Rally 2 Set to %f, %f, %f\n",CLUSTER_RALLY2.X, CLUSTER_RALLY2.Y, CLUSTER_RALLY2.Z);
}

double agent::getRally2X() {
    return CLUSTER_RALLY2.X;
}

double agent::getRally2Y() {
    return CLUSTER_RALLY2.Y;
}

double agent::getAgentAlt() {
    return AGENT_ALT;
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
    printf("parseCommand Results: \n");
    printf("Cluster: %d\n", *cluster);
    printf("Command: %d\n", *command);
};

void agent::setClusterLocalTarget(double x, double y, double z) {
    CLUSTER_LOCAL_TARGET.X = x;
    CLUSTER_LOCAL_TARGET.Y = y;
    CLUSTER_LOCAL_TARGET.Z = z;
    printf("Cluster local target set to: %f, %f, %f\n",x,y,z);
}

void agent::setClusterType(int type) {
    CLUSTER_TYPE = type;
    printf("Cluster changed to type %d\n",type);
}

void acknowledge(int agentID,int* command) {
    // agentAcknowledgement.type = NAMED_VALUE_FLOAT;
    // agentAcknowledgement.header.stamp = ros::Time::now();
    
    // //build and publish the acknowledgement
    // char str[10];
    // sprintf(str, "K%d#%d", agentID, *command);
    // agentAcknowledgement.name = str; 
    // ak_pub.publish(agentAcknowledgement);
}

double agent::getSysid() {
    return SYSID;
}

double agent::calculateXLineOffset() {
    double length = (CLUSTER_SIZE - 1) * CLUSTER_RADIUS;
    double offset = (CLUSTER_ORDER - 1) * length;
    offset = offset - length / 2; //Need to think about this one some more
    return offset;
}

double agent::calculateYLineOffset() {
    return 0;
}

int agent::getFlightMode() {
    return HRL_FLIGHTMODE;
}

double agent::getClusterid() {
    return CLUSTERID;
}

void agent::setPublish(bool logic) {
    PUBLISH = logic;
    printf("Publish status is: %d\n", PUBLISH);
}

bool agent::getPublish() {
    return PUBLISH;
}

#endif