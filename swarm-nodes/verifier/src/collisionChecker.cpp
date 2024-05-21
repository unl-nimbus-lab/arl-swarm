#include <iostream>
#include "ros/ros.h"
#include "safetyChecker.h"
#include "verifier_comms/hyperRectangle.h"
//This ros node is to store "my_rs" and compare
//it to the other agents' reachable sets

class collisionChecker {
    public:
        collisionChecker();

        ros::NodeHandle nh;

        hyperRectangle my_rs;
        hyperRectangle rcvd_rs;

        ros::Subscriber my_rs_subscriber;
        ros::Subscriber rcvd_rs_subscriber;

        void my_rs_cb(const verifier_comms::hyperRectangle::ConstPtr& msg);
        void rcvd_rs_cb(const verifier_comms::hyperRectangle::ConstPtr& msg);
};

void collisionChecker::my_rs_cb(const verifier_comms::hyperRectangle::ConstPtr& msg) {

}

void collisionChecker::rcvd_rs_cb(const verifier_comms::hyperRectangle::ConstPtr& msg) {
    rcvd_rs.disp();

}

collisionChecker::collisionChecker() {

    interval x(0.0,0.0);

    std::vector<interval> intervals;

    intervals.push_back(x);
    intervals.push_back(x);
    intervals.push_back(x);
    intervals.push_back(x);
    intervals.push_back(x);
    intervals.push_back(x);

    my_rs = hyperRectangle(intervals);
    rcvd_rs = hyperRectangle(intervals);

    my_rs_subscriber = nh.subscribe("/my_reach_state", 5, &collisionChecker::my_rs_cb, this);
    rcvd_rs_subscriber = nh.subscribe("/agent_2/recieved_rs", 30, &collisionChecker::rcvd_rs_cb, this);
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "collision_checker");
    collisionChecker cc;
    ros::spin();
    return 0;
}