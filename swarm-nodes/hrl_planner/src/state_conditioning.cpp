#include "ros/ros.h"
#include "swarm_comms/allAgentsPosStamped.h"

void state_update_cb(const swarm_comms::allAgentsPosStamped) {
//Update the state vector
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "state_conditioning");
    ros::NodeHandle nh;
    ros::Subscriber swarm_state_sub = nh.subscribe("/agent_pos", 10, state_update_cb);
}