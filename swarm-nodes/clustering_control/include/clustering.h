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

//HRL Flightmodes
#define DONT_PUBLISH 0
#define CLUSTER_GUIDED 1

//Cluster Formation types
#define CIRCLE_FORMATION 1
#define LINEAR_FORMATION 2

//Cluster Commands
#define PING 0
#define START_CLUSTER_GUIDED 1
#define STOP_CLUSTER_GUIDED 2
#define CHANGE_FLIGHT_MODE 3
#define CHANGE_POSITION 4
#define CHANGE_RADIUS 5
#define CHANGE_TYPE 6
#define SET_HOME_POSITION 7

//Agent Commands

#define CHANGE_CLUSTER_ID 2
#define CHANGE_SYSID 3
#define CHANGE_CLUSTER_ORDER 4
#define CHANGE_CLUSTER_SIZE 5
#define CHANGE_AGENT_ALTITUDE 6

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
        double getRallyX();
        double getRallyY();
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
        } CLUSTER_OFFSETS, AGENT_LOCAL_TARGET, CLUSTER_LOCAL_TARGET, CLUSTER_RALLY;
        //Cluster Offsets: How far (in meters this agent should be offset from the cluster center)
        //Agent Local Target: Local Agent target after offsets applied
        //Cluster Local Target: Location (in meters) the center of the cluster should be relative to "GLOBAL ORIGIN"
        struct {
            double LAT;
            double LON;
            double ALT;
        } AGENT_RALLY, GLOBAL_ORIGIN, GPS_OFFSETS;
        //Agent Rally: Not used
        //Cluster Rally: Not used
        //Global Origin: Common world frame for the swarm
};

void agent::setPositionTarget() {
    //This function is used to update the global target of the agent (pos_msg) 
    //based on the hrl flightmode, this is the only function that should
    //directly manipulate the pos_msg.
    double tempx, tempy, tempz;

    switch (HRL_FLIGHTMODE) {
        case CLUSTER_GUIDED: //Move the cluster in the local frame
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
        default:
            printf("INVALID FLIGHT MODE REQUESTED");
            break;
    };
}

void agent::setAgentLocalTarget(double x, double y, double z) {

    AGENT_LOCAL_TARGET.X = x;
    AGENT_LOCAL_TARGET.Y = y;
    AGENT_LOCAL_TARGET.Z = z;
    printf("Agent Local Target Set: %f, %f, %f\n", AGENT_LOCAL_TARGET.X, AGENT_LOCAL_TARGET.Y, AGENT_LOCAL_TARGET.Z);
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
            printf("INVALID CLUSTER TYPE ATTEMPTED");
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

void agent::setRally(double x, double y) {
    CLUSTER_RALLY.X = x;
    CLUSTER_RALLY.Y = y;
    CLUSTER_RALLY.Z = AGENT_ALT;
    printf("Cluster Rally Set to %f, %f, %f\n",CLUSTER_RALLY.X, CLUSTER_RALLY.Y, CLUSTER_RALLY.Z);
}

double agent::getRallyX() {
    return CLUSTER_RALLY.X;
}

double agent::getRallyY() {
    return CLUSTER_RALLY.Y;
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