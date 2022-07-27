#include "clustering.h"

agent hrlAgent;

int main(int argc, char **argv) {

    //read in command line arguments
    //TODO: this block is hideous, clean it up
    hrlAgent.setSysid(std::stod(argv[1]));
    hrlAgent.setClusterid(std::stod(argv[2]));
    hrlAgent.setClusterOrder(std::stod(argv[3]));
    hrlAgent.setClusterSize(std::stod(argv[4]));
    hrlAgent.setClusterRadius(std::stod(argv[5]));
    hrlAgent.setAgentAlt(std::stod(argv[6]));
    hrlAgent.setHome(std::stod(argv[7]), std::stod(argv[8]), std::stod(argv[9]));
    hrlAgent.setRally(std::stod(argv[10]),std::stod(argv[11]));

    //Set the initial position target: the cluster is centered around 
    //the rally point (post drop or takeoff)
    hrlAgent.setClusterOffsets();
    hrlAgent.setClusterLocalTarget(hrlAgent.getRallyX(),hrlAgent.getRallyY(),hrlAgent.getAgentAlt());
    hrlAgent.setPositionTarget();

    //ROS Code:
    ros::init(argc, argv, "clustering_control");
    ros::NodeHandle nh;
    ros::Subscriber liveHRLReference_sub = nh.subscribe("/mavros/debug_value/debug_vector", 10, HRLReference_cb);
    ros::Subscriber swarmState_sub = nh.subscribe("/mavros/global_position/swarm",10, FTL_cb);
    ak_pub = nh.advertise<mavros_msgs::DebugValue> ( "/mavros/debug_value/send", 10, true );
    wp_pub = nh.advertise<geographic_msgs::GeoPoseStamped> ( "/mavros/setpoint_position/global", 10, true );

   
    while (ros::ok()) {
        hrlAgent.pos_msg.header.stamp = ros::Time::now();
        wp_pub.publish( hrlAgent.pos_msg );
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    return 0;
}

void HRLReference_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    //over-the radio commands are handled here, the first step is to deconstruct the command
    //sent over the debug vector channel
    mavros_msgs::DebugValue HRLReference = *msg;
    int *targetEntity = new int;
    int *command = new int;
    char entity = HRLReference.name[0];  //[S]warm, [C]luster, or [A]gent
    parseCommand(HRLReference.name, targetEntity, command);

    /*Now that the target entitiy and command number are known, its time to do something with
    //the new information. In general, some private variable within the agent class is changed.
    Then, handler functions are used to ultimatlely update the agent.pos_msg (the commonly used
    function is agent::setPositionTarget */
        switch(entity) {
            case 'A':
                if (*targetEntity == hrlAgent.getSysid()) {
                    switch(*command) {
                        case 0: //Ping (just send an acknowledgement back)
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 1: //Set the SYSID (THIS IS VERY DANGROUS, MAYBE DONT USE!!!)
                            //hrlAgent.setSysid(HRLReference.data[0]);
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 2: //Set the cluster ID
                            hrlAgent.setClusterid(HRLReference.data[0]);
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 3: //set the HRL flight mode
                            hrlAgent.setFlightMode(HRLReference.data[0]);
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 4: //set the cluster order
                            hrlAgent.setClusterOrder(HRLReference.data[0]);
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 5: //Set the cluster size
                            hrlAgent.setClusterSize(HRLReference.data[0]);
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 6: //Set the agent altitude
                            hrlAgent.setAgentAlt(HRLReference.data[0]);
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 7: //TODO
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 8: //TODO
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        };
                }
                break;
            case 'C': //Cluster-level command
                if (*targetEntity == hrlAgent.getClusterid()) {
                    switch(*command) {
                        case 0: //Ping (just send an acknowedgemet back)
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 1:  //Change HRL_FLIGHTMODE to payload value
                            hrlAgent.setFlightMode(HRLReference.data[0]);
                            hrlAgent.setPositionTarget();
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 2: //Change CLUSTER_LOCAL_TARGET to payload
                            hrlAgent.setClusterLocalTarget(HRLReference.data[0],HRLReference.data[1],HRLReference.data[2]);
                            hrlAgent.setPositionTarget();
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 3: //Change CLUSTER_RADIUS to payload
                            hrlAgent.setClusterRadius(HRLReference.data[0]);
                            hrlAgent.setClusterOffsets();
                            hrlAgent.setPositionTarget();
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case 4: //Change the Cluster type
                            hrlAgent.setClusterType(HRLReference.data[0]);
                            hrlAgent.setClusterOffsets();
                            hrlAgent.setPositionTarget();
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                    };
                }
                break;
        };
}

void FTL_cb(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
    //This function updates the FTL Target struct and FTL Cluster offsets
    mavros_msgs::SwarmGPS Target = *msg;
    if (Target.system_id == hrlAgent.getFTLTarget()) {
        if (Target.latitude != 0 && Target.longitude != 0) { //Minimal protection against invalid target coords
            hrlAgent.setFTLTarget(Target.latitude,Target.longitude,Target.altitude);
            //Since we don't have a function that goes from lat/lon -> x/y meters, the FTL offsets are 
            //given in delta-latitude and delta-longitude from the FTL Target.
            hrlAgent.setGPSOffsets();
            hrlAgent.setPositionTarget();
            hrlAgent.FTL_TARGET_SET = true;
        }
    }
}

//TODO:
//ACKNOWLEDGEMENTS -- DONE? -- Kinda
//FTL -- DONE
//SUPPORT FOR LINEAR CLUSTER TYPE -- DONE
//CLEANUP STARTUP DOOED