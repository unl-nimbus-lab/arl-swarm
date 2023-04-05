#include "ros/ros.h"
#include "swarm_comms/allAgentsPosStamped.h"
#include "geometry_msgs/TwistStamped.h"

void velocity_update_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "state_conditioning");
    ros::NodeHandle nh;
    //ros::Subscriber swarm_state_sub = nh.subscribe("/agent_pos", 10, state_update_cb); NEED TO USE THIS ONE IN THE FUTURE
    
}