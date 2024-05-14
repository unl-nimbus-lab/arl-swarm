#include "verifier/verifier.h"

#define START_ROS 1
#define STOP_ROS 2

/* This node will subscribe to the agent's xy position topic and calculate it's reachable space */

agent::agent() {

    physical_state_subscriber = nh.subscribe("/agent_state",   1, &agent::state_update_cb, this);
    cyber_state_subscriber = nh.subscribe(   "/ncyber_state",   1, &agent::compute_reach_set_cb,  this);

    rect_publisher = nh.advertise<verifier_comms::hyperRectangle>("/my_reach_state", 1, true);

    //Switching to a timer based intterupt for reachable set computation
    rs_timer = nh.createTimer(ros::Duration(0.1), &agent::compute_rs, this);

    nh.param<int>("SystemID", SYSID_THISMAV, 0);
    ROS_WARN("Set SYSID_THISMAV to %d", SYSID_THISMAV);

    nh.param<float>("reachTimout", reachTimeout, 0.005);
    ROS_WARN("Set reachTimeout to %f", reachTimeout);


}

void agent::compute_rs( const ros::TimerEvent& event ) {
            
    //std::cout << "computing a reach set for " << reachTimeout << " seconds" << std::endl;
    float compute_time = reachTimeout * 0.4;

    //The actual time it takes to compute the reachable set
    double actualTime = getUnixTime(); 

    computeReachableSet(1.5, 0.5, 4.0, 4.0, compute_time);

    /*
    Because I often forget compute reachable set arguments:
    1) Postition Error (minimum space size)
    2) Velocity Error (minimum increase)
    3) Reach time (how far ahead to look)
    4) Initial Step Size (how big )
    5) Reach timeout
    */

    actualTime = getUnixTime() - actualTime;

    packRectangleMessage(myReachableState, rsMessage);
    rsMessage.header.stamp = ros::Time::now();

    ros::Time tempTime(actualTime);
    rsMessage.header.stamp = ros::Time::now();
    rsMessage.rs_time = tempTime;

    rect_publisher.publish(rsMessage);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "verifier");

    agent thisAgent;

    ros::spin();

    return 0;
}
