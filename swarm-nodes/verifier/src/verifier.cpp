#include "verifier/verifier.h"

#define START_ROS 1
#define STOP_ROS 2

/* This node will subscribe to the agent's xy position topic and calculate it's reachable space */

agent::agent() {

    physical_state_subscriber = nh.subscribe("/agent_state",   1, &agent::state_update_cb, this);
    cyber_state_subscriber = nh.subscribe(   "/ncyber_state",   1, &agent::compute_reach_set_cb,  this);
    reach_time_subscriber = nh.subscribe(    "/reach_timeout",     1, &agent::reach_timeout_cb,        this);

    rect_publisher = nh.advertise<verifier_comms::hyperRectangle>("/my_reach_state", 1, true);


    //Switching to a timer based intterupt for reachable set computation
    rs_timer = nh.createTimer(ros::Duration(0.1), &agent::compute_rs, this);

    nh.param<int>("SystemID", SYSID_THISMAV, 0);
    ROS_WARN("Set SYSID_THISMAV to %d", SYSID_THISMAV);

    nh.param<float>("reachTimout", reachTimeout, 0.005);
    ROS_WARN("Set reachTimeout to %f", reachTimeout);

    //Logging Publishers
    rs_pub = nh.advertise<std_msgs::Float32>("/rs_time", 1, true); //For publishing the rs_time
    en_pub = nh.advertise<std_msgs::Float32>("/en_time", 1, true); //For publishing the en_time

    rsMessage.id = SYSID_THISMAV;

}

void agent::compute_rs( const ros::TimerEvent& event ) {

    std_msgs::Float32 rs_time;
    std_msgs::Float32 en_time;
            
    //std::cout << "computing a reach set for " << reachTimeout << " seconds" << std::endl;
    float compute_time = reachTimeout * 0.4;

    //The actual time it takes to compute the reachable set
    double start_compute = getUnixTime(); 

    computeReachableSet(1.5, 0.5, reachTime, 4.0, compute_time);

    /*
    Because I often forget compute reachable set arguments:
    1) Postition Error (minimum space size)
    2) Velocity Error (minimum increase)
    3) Reach time (how far ahead to look
    4) Initial Step Size (how big )
    5) Reach timeout
    */
    
    //finish timing the computation
    double end_compute = getUnixTime();

    rs_time.data = end_compute - start_compute;

    //Populate the ROS message with the reachable set
    packRectangleMessage(myReachableState, rsMessage);
    
    rsMessage.header.stamp = ros::Time::now();
    ros::Time tempTime(start_compute + 4.0);

    //Find the encode time
    double end_encode = getUnixTime();
    en_time.data = end_encode - end_compute;


    //Add the timestamp and the actual time
    rsMessage.header.stamp = ros::Time::now();
    rsMessage.rs_time = tempTime;

    rect_publisher.publish(rsMessage);
    rs_pub.publish(rs_time);
    en_pub.publish(en_time);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "verifier");
    agent thisAgent;
    ros::spin();
    return 0;
}
