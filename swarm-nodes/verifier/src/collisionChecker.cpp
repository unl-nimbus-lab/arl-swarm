#include <iostream>
#include "ros/ros.h"
#include "safetyChecker.h"
#include "verifier_comms/hyperRectangle.h"
#include "mavros_msgs/SetMode.h"
#include "std_msgs/Bool.h"
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
        ros::Subscriber collision_action_subscriber;

        ros::ServiceClient set_mode;

        mavros_msgs::SetMode brake_msg;
        mavros_msgs::SetMode guided_msg;

        bool collision_action_enabled;
        std_msgs::Bool collision_action_msg;

        void my_rs_cb(const verifier_comms::hyperRectangle::ConstPtr& msg);
        void rcvd_rs_cb(const verifier_comms::hyperRectangle::ConstPtr& msg);
        void collision_action(const std_msgs::Bool::ConstPtr& msg);
};

void collisionChecker::my_rs_cb(const verifier_comms::hyperRectangle::ConstPtr& msg) {
    my_rs.intervals[0].min = msg->x.min;
    my_rs.intervals[0].max = msg->x.max;
    my_rs.intervals[1].min = msg->y.min;
    my_rs.intervals[1].max = msg->y.max;
    my_rs.intervals[2].min = msg->z.min;
    my_rs.intervals[2].max = msg->z.max;
    //my_rs.disp();

}

void collisionChecker::rcvd_rs_cb(const verifier_comms::hyperRectangle::ConstPtr& msg) {
    rcvd_rs.intervals[0].min = msg->x.min;
    rcvd_rs.intervals[0].max = msg->x.max;
    rcvd_rs.intervals[1].min = msg->y.min;
    rcvd_rs.intervals[1].max = msg->y.max;
    rcvd_rs.intervals[2].min = msg->z.min;
    rcvd_rs.intervals[2].max = msg->z.max;
    //rcvd_rs.disp();

    if (my_rs.intersects(rcvd_rs)) {
        std::cout << "Collision detected" << std::endl;
        if (collision_action_enabled) {
            set_mode.call(brake_msg);
        }
    }
}

void collisionChecker::collision_action(const std_msgs::Bool::ConstPtr& msg) {
    ROS_WARN("Collision action set to: %d", msg->data);
    collision_action_enabled = msg->data;
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

    collision_action_enabled = true;

    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    brake_msg.request.custom_mode = "BRAKE";
    guided_msg.request.custom_mode = "GUIDED";

    my_rs_subscriber = nh.subscribe("/my_reach_state", 5, &collisionChecker::my_rs_cb, this);
    rcvd_rs_subscriber = nh.subscribe("/recieved_rs", 30, &collisionChecker::rcvd_rs_cb, this);
    collision_action_subscriber = nh.subscribe("/collision_action", 5, &collisionChecker::collision_action, this);

}

int main (int argc, char **argv) {
    ros::init(argc, argv, "collision_checker");
    collisionChecker cc;
    ros::spin();
    return 0;
}