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

    hrlAgent.displayCommandlineArguments();

    //Set the initial position target: the cluster is centered around 
    //the rally point (post drop or takeoff)
    hrlAgent.setClusterOffsets();
    hrlAgent.setPositionTarget();

    //ROS Code:
    ros::init(argc, argv, "clustering_control");
    ros::NodeHandle nh;
    ros::Subscriber liveHRLReference_sub = nh.subscribe("/mavros/debug_value/debug_vector", 10, HRLReference_cb);
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
    mavros_msgs::DebugValue HRLReference = *msg;
    int *targetEntity = new int;
    int *command = new int;
    char entity = HRLReference.name[0];  //[S]warm, [C]luster, or [A]gent
    parseCommand(HRLReference.name, targetEntity, command);
        switch(entity) {
            case 'C': //Cluster-level command
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
                break;
        };
}


//TODO:
//ACKNOWLEDGEMENTS -- DONE?
//FTL
//SUPPORT FOR LINEAR CLUSTER TYPE
//CLEANUP STARTUP