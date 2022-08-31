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
    hrlAgent.setRally1(std::stod(argv[10]),std::stod(argv[11]));
    hrlAgent.setRally2(std::stod(argv[12]),std::stod(argv[13]));

    //Set the initial position target: the cluster is centered around 
    //the rally point (post drop or takeoff)
    hrlAgent.setClusterOffsets();
    hrlAgent.setClusterLocalTarget(hrlAgent.getRally1X(),hrlAgent.getRally1Y(),hrlAgent.getAgentAlt());
    hrlAgent.setPositionTarget();

    //ROS-Specific items:
    ros::init(argc, argv, "clustering_control");
    ros::NodeHandle nh;
    ros::Subscriber liveHRLReference_sub = nh.subscribe("/mavros/debug_value/debug_vector", 10, HRLReference_cb);
    ros::Subscriber globalPosition_sub = nh.subscribe("/mavros/global_position/swarm",1, GPS_cb);
    ak_pub = nh.advertise<mavros_msgs::DebugValue> ( "/mavros/debug_value/send", 10, true );
    wp_pub = nh.advertise<geographic_msgs::GeoPoseStamped> ( "/mavros/setpoint_position/global", 10, true );

    while (ros::ok()) {
        if (hrlAgent.getPublish() == true) {
            hrlAgent.pos_msg.header.stamp = ros::Time::now();
            if (hrlAgent.getPublish())
                wp_pub.publish( hrlAgent.pos_msg );
        }
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
    ROS_INFO("DEBUG COMMAND PARSED");

    /*Now that the target entitiy and command number are known, its time to do something with
    //the new information. In general, some private variable within the agent class is changed.
    Then, handler functions are used to ultimatlely update the agent.pos_msg (the commonly used
    function is agent::setPositionTarget */
        switch(entity) {
            case 'A':
                // if (*targetEntity == hrlAgent.getSysid()) {
                //     switch(*command) {
                //         case PING: //Ping (just send an acknowledgement back)
                //             acknowledge(hrlAgent.getSysid(), command);
                //             break;
                //         case CHANGE_SYSID: //Set the SYSID (THIS IS VERY DANGROUS, MAYBE DONT USE!!!)
                //             //hrlAgent.setSysid(HRLReference.data[0]);
                //             acknowledge(hrlAgent.getSysid(), command);
                //             break;
                //         case CHANGE_CLUSTER_ID: //Set the cluster ID
                //             hrlAgent.setClusterid(HRLReference.data[0]);
                //             acknowledge(hrlAgent.getSysid(), command);
                //             break;
                //         case CHANGE_FLIGHT_MODE: //set the HRL flight mode
                //             hrlAgent.setFlightMode(HRLReference.data[0]);
                //             acknowledge(hrlAgent.getSysid(), command);
                //             break;
                //         case CHANGE_CLUSTER_ORDER: //set the cluster order
                //             hrlAgent.setClusterOrder(HRLReference.data[0]);
                //             acknowledge(hrlAgent.getSysid(), command);
                //             break;
                //         case CHANGE_CLUSTER_SIZE: //Set the cluster size
                //             hrlAgent.setClusterSize(HRLReference.data[0]);
                //             acknowledge(hrlAgent.getSysid(), command);
                //             break;
                //         case CHANGE_AGENT_ALTITUDE: //Set the agent altitude
                //             hrlAgent.setAgentAlt(HRLReference.data[0]);
                //             acknowledge(hrlAgent.getSysid(), command);
                //             break;
                //         case 7: //TODO
                //             acknowledge(hrlAgent.getSysid(), command);
                //             break;
                //         case 8: //TODO
                //             acknowledge(hrlAgent.getSysid(), command);
                //             break;
                //         };
                // }
                break;
            case 'C': //Cluster-level command
                ROS_INFO("Got a cluster-level command\n");
                if (*targetEntity == hrlAgent.getClusterid()) {
                    ROS_INFO("Got a cluster-level command for this agent");
                    switch(*command) {
                        case PING: //Ping (just send an acknowedgemet back)
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case CHANGE_FLIGHT_MODE:  //Change HRL_FLIGHTMODE to payload value
                            hrlAgent.setFlightMode(HRLReference.data[0]);
                            hrlAgent.setPositionTarget();
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case SET_CLUSTER_LOCAL_POSITION: //Change CLUSTER_LOCAL_TARGET to payload
                            hrlAgent.setClusterLocalTarget(HRLReference.data[0],HRLReference.data[1],HRLReference.data[2]);
                            hrlAgent.setPositionTarget();
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case SET_CLUSTER_RADIUS: //Change CLUSTER_RADIUS to payload
                            hrlAgent.setClusterRadius(HRLReference.data[0]);
                            hrlAgent.setClusterOffsets();
                            hrlAgent.setPositionTarget();
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case SET_CLUSTER_TYPE: //Change the Cluster type
                            hrlAgent.setClusterType(HRLReference.data[0]);
                            hrlAgent.setClusterOffsets();
                            hrlAgent.setPositionTarget();
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case START_CLUSTER_GUIDED:
                            hrlAgent.setPublish(true);
                            acknowledge(hrlAgent.getSysid(), command);
                            break;  
                        case STOP_CLUSTER_GUIDED:
                            hrlAgent.setPublish(false);
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case SET_HOME_POSITION:
                            hrlAgent.setHome(HRLReference.data[0],HRLReference.data[1],HRLReference.data[2]);
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case SET_CLUSTER_GLOBAL_POSITION:
                            hrlAgent.setClusterGlobalTarget(HRLReference.data[0],HRLReference.data[1],HRLReference.data[2]);
                            hrlAgent.setPositionTarget();
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case SEPARATE_ALTITUDES:
                            if (hrlAgent.CURRENT_POS.LAT != 0) {
                                printf("Separating Altidutes\n");
                                hrlAgent.setFlightMode(ALTITUDE_SEP_MODE);
                                hrlAgent.setPositionTarget();
                            }
                            else {
                                printf("Not separating, No GPS reading\n");
                            }
                            break;
                        default:
                            printf("INVALID COMMAND NUMBER, COMMAND IGNORED\n");
                            break;
                    };
                }
                break;
            default:
                printf("INVALID COMMAND ID, SPECIFIY [S]WARM, [C]LUSTER, OR [A]GENT IN THE DUBG STRING\n");
            break;
        };
}

void GPS_cb(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
    mavros_msgs::SwarmGPS gps_msg = *msg;
    if(gps_msg.system_id == hrlAgent.getSysid()) {
            hrlAgent.CURRENT_POS.LAT = gps_msg.latitude;
            hrlAgent.CURRENT_POS.LON = gps_msg.longitude;
            hrlAgent.CURRENT_POS.ALT = gps_msg.altitude;
    }
}
//TODO:
//ACKNOWLEDGEMENTS -- DONE -- TESTED
//FTL -- DONE -- EDITS -- REMOVE -- DONE
//SUPPORT FOR LINEAR CLUSTER TYPE -- DONE
//SUPPORT FOR FLYING V CLUSTER TYPE
//POTENTIALY REMOVE ACKNOWLEDGEMENT  -- DONE
//CHANGE THIS PUB TO NEW TOPIC, SOMETHING LIKE 
// /agent_command_pos --> (global_oublisher) --> /mavros_setpoint_position/global

//MINIMAL FUNCTIONALITY
//CLUSTER:
//START HRL COMMANDS/PUBLISHING -- NEEDS TESTED -- TESTED
//STOP HRL COMMANDS/PUBLISHING -- NEEDS TESTED -- TESTED
//UPDATE CLUSTER TARGET POSITION IN LOCAL FRAME -- NEEDS TESTED -- TESTED
//SET HOME POSITION MANUALLY -- NEEDS TESTED -- TESTED

//NICE TO HAVE FUNCTIONALITY
//SET HOME POSITION FROM LEADER
//FOLLOW THE LEADER

//AGENT:
//START HRL COMMANDS/PUBLISHING
//STOP HRL COMMANDS/PUBLISHING
//UPDATE AGENT TARGET POSITION IN LOCAL FRAME
//UPDATE AGENT TARGET POSIITON IN GLOBAL FRAME
