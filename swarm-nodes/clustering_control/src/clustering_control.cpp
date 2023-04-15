#include "clustering.h"

agent hrlAgent;

int main(int argc, char **argv) {
    //read in command line arguments
    //TODO: this block is hideous, clean it up
    hrlAgent.setSysid(std::stod(argv[1])); //The ardupilot SYSID_THISMAV parameter
    hrlAgent.setClusterid(std::stod(argv[2]));  //Cluster ID parameter (should always == 1)
    hrlAgent.setClusterOrder(std::stod(argv[3])); 
    hrlAgent.setOriginalClusterOrder(std::stod(argv[3]));
    hrlAgent.setClusterSize(std::stod(argv[4]));
    hrlAgent.setClusterRadius(std::stod(argv[5]));
    hrlAgent.setAgentAlt(std::stod(argv[6]));
    hrlAgent.setHome(std::stod(argv[7]), std::stod(argv[8]), std::stod(argv[9]));
    hrlAgent.setRally1(std::stod(argv[10]),std::stod(argv[11]));
    hrlAgent.setRally2(std::stod(argv[12]),std::stod(argv[13]));
    std::string filename = "/home/catkin_ws/src/clustering_control/src/sys_mats_2_2_best.json";

    //Set the initial position target: the cluster is centered around 
    //the rally point (post drop or takeoff)
    hrlAgent.setClusterOffsets(hrlAgent.getOriginalClusterOrder());
    hrlAgent.setClusterLocalTarget(hrlAgent.getRally1X(),hrlAgent.getRally1Y(),hrlAgent.getAgentAlt());
    hrlAgent.setPositionTarget();

    //Read in JSON swarm configs from JSON files
    json_reader jsonReader;

    jsonReader.readJSON(filename);

    //Setup ROS
    ros::init(argc, argv, "clustering_control");
    ros::NodeHandle nh;

    //ROS subscribers
    ros::Subscriber liveHRLReference_sub = nh.subscribe("/mavros/debug_value/debug_vector", 10, HRLReference_cb);
    //ros::Subscriber brake_sub = nh.subscribe("/mavros/debug_value/named_value_int", 10, brake_cb);    
    ros::Subscriber globalPosition_sub = nh.subscribe("/mavros/global_position/swarm",1, GPS_cb);

    //ROS Publishers
    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ak_pub = nh.advertise<mavros_msgs::DebugValue> ( "/mavros/debug_value/send", 10, true );
    wp_pub = nh.advertise<geographic_msgs::GeoPoseStamped> ( "/mavros/setpoint_position/global", 10, true );

    //main loop
    while (ros::ok()) {
        //////////// BEGIN HRL PERIODIC CALCULATIONS ////////////

        //////////// END HRL PERIODIC CALCULATIONS ////////////

        if (hrlAgent.getPublish() == true) { //Flag that indicates if the pos_msg should be published
            if (hrlAgent.CIRCLE_FLAG == true) {
                if (tick_count >= hrlAgent.getCircleTicks()) {//Circle rate 
                    //Update the cluster order
                    double order = hrlAgent.getClusterOrder();
                    if (order == hrlAgent.getClusterSize()) {
                        //Reset the order to 1 if the agent is at the top of the cluster order
                        hrlAgent.setClusterOrder(1);
                    }
                    else {
                        //If this is not at the top of the cluster order, just add one
                        hrlAgent.setClusterOrder(order + 1);
                    }
                    //Update the cluster offsets after the order has changed
		            printf("Cluster order: %f\n",hrlAgent.getClusterOrder());
                    hrlAgent.setClusterOffsets(hrlAgent.getClusterOrder());
                    hrlAgent.setPositionTarget();
                    //reset the counter
                    tick_count = 0;
                }
                else {
                    tick_count++;
                }
            }
            if (hrlAgent.FTL_ENABLE == true) {
                hrlAgent.setPositionTarget();
                printf("FTL ENABLE is true\n");
            }

            hrlAgent.pos_msg.header.stamp = ros::Time::now(); //Add the header to the position target message
            wp_pub.publish( hrlAgent.pos_msg ); //publish the position target message
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
                if (*targetEntity == hrlAgent.getSysid()) {
                    switch(*command) {
                        case SET_CLUSTER_OFFSETZ:
                            //Change how high this agent is
                            hrlAgent.setClusterOffsetZ(HRLReference.data[0]);
                            hrlAgent.setPositionTarget();
                            break;
                        case CHANGE_FLIGHT_MODE:  
                            //Change HRL_FLIGHTMODE to payload value
                            hrlAgent.setFlightMode(HRLReference.data[0]);
                            hrlAgent.setPositionTarget();
                            break;
                        case SET_AGENT_LOCAL_TARGET:
                            //Change where the agent is in the local frame
                            hrlAgent.setAgentLocalTarget(HRLReference.data[0],HRLReference.data[1],HRLReference.data[2]);
                            hrlAgent.setPositionTarget();
                            break;
                        case SET_AGENT_GLOBAL_TAGET:
                            //change where the agent is in the global frame
                            hrlAgent.setAgentGlobalTarget(HRLReference.data[0],HRLReference.data[1],HRLReference.data[2]);
                            hrlAgent.setPositionTarget();
                            break;
                        case SET_BRAKE:
                            std::cout << "Braking" << std::endl;
                            mode_msg.request.custom_mode = "BRAKE";
                            set_mode.call(mode_msg);
                            break;
                    }
                }
                break;
            case 'C': //Cluster-level command
                if (*targetEntity == hrlAgent.getClusterid()) {
                    switch(*command) {
                        case PING: 
                            //Ping (just send an acknowedgemet back)
                            acknowledge(hrlAgent.getSysid(), command);
                            break;
                        case CHANGE_FLIGHT_MODE:  
                            //Change HRL_FLIGHTMODE to payload value
                            hrlAgent.setFlightMode(HRLReference.data[0]);
                            hrlAgent.setPositionTarget();
                            break;
                        case SET_CLUSTER_LOCAL_POSITION: 
                            //Change CLUSTER_LOCAL_TARGET to payload
                            hrlAgent.setClusterLocalTarget(HRLReference.data[0],HRLReference.data[1],HRLReference.data[2]);
                            hrlAgent.setPositionTarget();
                            break;
                        case SET_CLUSTER_RADIUS:
                            //Change CLUSTER_RADIUS to vector payload
                            hrlAgent.setClusterRadius(HRLReference.data[0]);
                            hrlAgent.setClusterOffsets(hrlAgent.getOriginalClusterOrder());
                            hrlAgent.setPositionTarget();
                            break;
                        case SET_CLUSTER_TYPE:
                            //Change the Cluster type/formation style
                            hrlAgent.setClusterType(HRLReference.data[0]);
                            hrlAgent.setClusterOffsets(hrlAgent.getOriginalClusterOrder());
                            hrlAgent.setPositionTarget();
                            break;
                        case START_CLUSTER_GUIDED:
                            //Start publishing to mavros topics
                            hrlAgent.setPublish(true);
                            break;  
                        case STOP_CLUSTER_GUIDED:
                            //Stop all publishing to the mavros topics
                            hrlAgent.setPublish(false);
                            break;
                        case SET_HOME_POSITION:
                            //Update the cluster's global origin
                            hrlAgent.setHome(HRLReference.data[0],HRLReference.data[1],HRLReference.data[2]);
                            break;
                        case SET_CLUSTER_GLOBAL_POSITION:
                            hrlAgent.setClusterGlobalTarget(HRLReference.data[0],HRLReference.data[1],HRLReference.data[2]);
                            hrlAgent.setPositionTarget();
                            break;
                        case SET_FTL_SYSID:
                            //Chose which sysid the cluster will follow
                            hrlAgent.setFTLSysid(HRLReference.data[0]);
                            break;
                        case ENABLE_CIRCLE:
                            //Start Circling
                            hrlAgent.enableCircle();
                            break;
                        case DISABLE_CIRCLE:
                            //Stop Circling
                            hrlAgent.disableCircle();
                            break;
                        case SET_CIRCLE_RATE:
                            //Change the Circle Rate
                            hrlAgent.setCircleTicks(HRLReference.data[0]);
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
    if (gps_msg.system_id == hrlAgent.getSysid()) {
        hrlAgent.setCurrentPos(gps_msg.latitude,gps_msg.longitude,gps_msg.altitude);
    }
    if (gps_msg.system_id == hrlAgent.getFTLSysid()) {
        hrlAgent.setFTLloc(gps_msg.latitude,gps_msg.longitude,gps_msg.altitude);
    }
}
