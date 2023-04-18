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
    std::string filename = "/home/catkin_ws/src/clustering_control/src/sys_mats_4_4_best.json";

    //Set the initial position target: the cluster is centered around 
    //the rally point (post drop or takeoff)
    hrlAgent.setClusterOffsets(hrlAgent.getOriginalClusterOrder());
    hrlAgent.setClusterLocalTarget(hrlAgent.getRally1X(),hrlAgent.getRally1Y(),hrlAgent.getAgentAlt());
    hrlAgent.setPositionTarget();

    //Read in JSON swarm configs from JSON files
    json_reader jsonReader;
    struct Sys system_matricies = jsonReader.readJSON(filename);

    //Begin Matrix Shennanigins

    //New Variables for HRL planner
    // THIS NEEDS TO NOT BE HARDCODED >> Change this to be output from MATLAB and defined globally
    MatrixXd swarm_comp(2,4); //Each cluster is a row

    // THIS NEEDS TO NOT BE HARDCODED >> Change this to be output from MATLAB
    MatrixXd index_encodings(2,4);

    int numberOfHRLStates = system_matricies.DTA.rows();
    int numberOfHRLAgents = system_matricies.p.sum();

    hrl_state.resize(numberOfHRLStates, 1); //The HRL State, formatted according to swarmComp.m
    raw_hrl_state.resize(numberOfHRLAgents*2,1);

    MatrixXd HRLxCurrent(numberOfHRLStates,1);
    MatrixXd HRLxCurrentLast(numberOfHRLStates,1);
    MatrixXd HRLx0(numberOfHRLStates,1);

    MatrixXd U(system_matricies.DTB.cols(),1);
    MatrixXd tempX0(numberOfHRLStates,1);
    MatrixXd tempXCurrent(numberOfHRLStates,1);
    MatrixXd tempXCurrentLast(numberOfHRLStates,1);

    MatrixXd tempX0Agent(2,1);
    MatrixXd tempXCurrentAgent(2,1);
    MatrixXd tempXCurrentLastAgent(2,1);
    MatrixXd agent_refs(2,1);

    MatrixXd targets_rel(2,3);
    MatrixXd tmpXCurrentAgent(1,2);
    MatrixXd tmpTarget_rel(1,2);
    MatrixXd tmpDist(2,1);

    swarm_comp << 1, 2, 3, 4, 5, 6, 7, 8;
    index_encodings << 0, 4, 8, 12, 24, 28, 32, 36;
    targets_rel << 24, 87, 5, 70, 58, 5;

    int max_count = 8;
    int current_count = 0;

    int myIndex = getHRLIndex(hrlAgent.getSysid(), swarm_comp, index_encodings);
    int myCluster = getHRLCluster(hrlAgent.getSysid(), swarm_comp);
    bool arrived = false;

    //Setup ROS
    ros::init(argc, argv, "clustering_control");
    ros::NodeHandle nh;

    //ROS subscribers
    ros::Subscriber liveHRLReference_sub = nh.subscribe("/mavros/debug_value/debug_vector", 10, HRLReference_cb);
    //ros::Subscriber brake_sub = nh.subscribe("/mavros/debug_value/named_value_int", 10, brake_cb);    
    ros::Subscriber globalPosition_sub = nh.subscribe("/mavros/global_position/swarm",1, GPS_cb);
    ros::Subscriber localPosition_sub = nh.subscribe("/agent_pos",1, stateUpdate_cb);

    //ROS Publishers
    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ak_pub = nh.advertise<mavros_msgs::DebugValue> ( "/mavros/debug_value/send", 10, true );
    wp_pub = nh.advertise<geographic_msgs::GeoPoseStamped> ( "/mavros/setpoint_position/global", 10, true );
    debug_pub = nh.advertise<std_msgs::Float64MultiArray> ( "/state_debug", 10, true);

    //main loop
    while (ros::ok()) {

        if (hrlAgent.getFlightMode() == HRL_NAVIGATE) {
            //////////// BEGIN HRL PERIODIC CALCULATIONS ////////////
            if (current_count == max_count) { //hacky way to set the HRL rate without disturbing the clustering control update rate

                //Check for arrival condition
                if (arrived == false) {

                    //1. Collect all the agent states
                    hrlStateUpdate(swarm_comp, index_encodings, raw_hrl_state);
                    HRLx0 = hrl_state;

                    HRLx0 = system_matricies.T*HRLx0; //CRITICAL: transfrom into new coordinate system
                    HRLxCurrent = HRLx0;
                    //2. Propogate HRL states until we find a rec >6 m away from current location
                    bool met_dist_req = false;
                    while ( (!met_dist_req) && (arrived == false) ) {
                        HRLxCurrentLast = HRLxCurrent;
                        //compute input for planner
                        U = -1*system_matricies.KL*(HRLxCurrent - system_matricies.aug_state);

                        //propogate HRL states
                        HRLxCurrent = system_matricies.DTA*(HRLxCurrent - system_matricies.aug_state) + system_matricies.DTB*U - system_matricies.ZD*hrl_sampling_time + system_matricies.aug_state;

                        // ////////// USE THIS BLOCK TO DEBUG STATE VECTORS //////////
                        // debugMsg.data.resize(HRLxCurrent.size());
                        // memcpy(&debugMsg.data[0], HRLxCurrent.data(), HRLxCurrent.size() * sizeof(double));
                        // debug_pub.publish(debugMsg);
                        // ////////// USE THIS BLOCK TO DEBUG STATE VECTORS //////////

                        //check to see if HRL is converging, in which case we're near the target
                        //un-transform
                        tempX0 = ((system_matricies.T).inverse() ) * HRLx0;
                        tempXCurrent = ((system_matricies.T).inverse() ) * HRLxCurrent;
                        tempXCurrentLast = ((system_matricies.T).inverse() ) * HRLxCurrentLast;
                        //extract the relevant states from each tempX... matrix 
                        tempX0Agent(0,0) = tempX0(myIndex    , 0); 
                        tempX0Agent(1,0) = tempX0(myIndex + 1, 0);
                        tempXCurrentAgent(0,0) = tempXCurrent(myIndex    , 0);
                        tempXCurrentAgent(1,0) = tempXCurrent(myIndex + 1, 0);
                        tempXCurrentLastAgent(0,0) = tempXCurrentLast(myIndex    , 0);
                        tempXCurrentLastAgent(1,0) = tempXCurrentLast(myIndex + 1, 0);

                        double converge = (tempXCurrentAgent - tempXCurrentLastAgent).squaredNorm();
                        if (converge < convergent_dist) {//Check for convergence
                            met_dist_req = true;
                            ROS_WARN("Met Dist Requirement from converge check");
                            agent_refs = tempXCurrentAgent; //I don't think this is necessary
                            //ROS: update the position target class variable
                            hrlAgent.setClusterLocalTarget(agent_refs(0,0), agent_refs(1,0), 0);
                        }
                        //Otherwise, compute the distance away from initial state
                        double dist = (tempXCurrentAgent - tempX0Agent).squaredNorm();
                        if (abs(dist) > waypoint_dist_req) {
                            met_dist_req = true;
                            ROS_WARN("Met Dist Requirement from waypoint dist requirement");
                            agent_refs = tempXCurrentAgent; //I don't think this is necessary
                            //ROS: update the position target class variable
                            hrlAgent.setClusterLocalTarget(agent_refs(0,0), agent_refs(1,0), 0);
                        }

                    } //end while
                    //3. Propogate each agent (not going to do anyting as per instructions)

                    //4. Check to see if we've arrived at the final waypoint
                    if (arrived == false) {
                        tmpXCurrentAgent(0,0) = hrl_state(myIndex     ,0);
                        tmpXCurrentAgent(0,1) = hrl_state(myIndex + 1 ,0);
                        tmpTarget_rel(0,0) = targets_rel(myCluster, 0);
                        tmpTarget_rel(0,1) = targets_rel(myCluster, 1);

                        double dist = (tmpTarget_rel - tmpXCurrentAgent).squaredNorm();
                        if (abs(dist) <= final_dist_req) {
                            ROS_WARN("Arrived");
                            arrived = true;
                        } else {
                            arrived = false;
                        }
                    }
                }
                hrlAgent.setPositionTarget(); //Update the position target ROS Message
                current_count = 0; //Reset the "HRL counter"
            } else {
                current_count++;
            }
            //////////// END HRL PERIODIC CALCULATIONS ////////////
        }

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

void stateUpdate_cb(const swarm_comms::allAgentsPosStamped::ConstPtr& msg) {
    swarm_comms::agentPos tempMsg;
    int raw_hrl_index;

    int numberOfAgents = msg->agents.size();
    for (int i = 0; i < numberOfAgents; i++) {
        tempMsg = msg->agents[i]; //For some reason I can't pull the ID without first assigning it to temporary message
        raw_hrl_index = (double(tempMsg.id) - 1) * 2;
        
        raw_hrl_state( raw_hrl_index, 0) = msg->agents[i].x;
        raw_hrl_state( raw_hrl_index + 1, 0) = msg->agents[i].y;
    }

}