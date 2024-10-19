#include <iostream>
#include "ros/ros.h"
#include "safetyChecker.h"
#include "verifier_comms/hyperRectangle.h"
#include "verifier/verifier.h"
#include "mavros_msgs/SetMode.h"
#include "std_msgs/Bool.h"
#include <algorithm>

#include "std_msgs/Float32.h"

#include <thread>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>


//This ros node is to store "my_rs" and compare
//it to the other agents' reachable sets

class collisionChecker {
    public:
        collisionChecker();

        ros::NodeHandle nh;

        int SYSID_THISMAV = 0;
        std::vector<int> colliding_mavs;

        int mode = 0;

        hyperRectangle my_rs;
        hyperRectangle rcvd_rs;

        ros::Subscriber my_rs_subscriber;
        ros::Subscriber rcvd_rs_subscriber;
        ros::Subscriber collision_action_subscriber;

        ros::Publisher dc_pub;
        ros::Publisher in_pub;

        ros::ServiceClient set_mode;

        mavros_msgs::SetMode brake_msg;
        mavros_msgs::SetMode guided_msg;

        bool collision_action_enabled;
        std_msgs::Bool collision_action_msg;

        bool collision_avoidance_taken = false;

        void my_rs_cb(const verifier_comms::hyperRectangle::ConstPtr& msg);
        void rcvd_rs_cb(const verifier_comms::hyperRectangle::ConstPtr& msg);
        void collision_action(const std_msgs::Bool::ConstPtr& msg);

        //Fancy threading variables
        std::queue<int> idQ;                    //shared queue of higher-priority agents that will collide
        std::mutex queueMutex;                  //Mutex to protect the queue
        std::condition_variable conditionVar;   //Condition variable to notify threads
        std::thread brakeThread;

        bool isInQueue(std::queue<int>& q, int value, std::mutex& mtx) {
            //lock the mutex
            std::lock_guard<std::mutex> lock(mtx);

            //Make a copy of the queue to safely inspect its contents
            std::queue<int> tempQueue = q;

            //Iterate through the copied queue
            while (!tempQueue.empty()) {
                if (tempQueue.front() == value) {
                    return true; // Value has been found in the queue
                }
                tempQueue.pop();
            }

            return false; //Value not found
        }

        void addToQueue(std::queue<int>& q, int value, std::mutex& mtx) {
            /*
            Add an integer to a queue

            q: the queue that is being changed
            value: the value to be added
            mtx: the mutex to protect the queue 
            */
            
            
            //lock the mutex
            std::lock_guard<std::mutex> lock(mtx);

            //Add the integer to the queue
            q.push(value);
        }

        bool isQueueEmpty(std::queue<int>& q, std::mutex& mtx) {
            /*
            Safely check is a queue is empty
            */

            //lock the mutex
            std::lock_guard<std::mutex> lock(mtx);

            //Check the queue
            return q.empty();
        }

        void removeFromQueue(std::queue<int>& q, std::mutex& mtx) {
            /*
            Safely remove from the queue, does not return the value
            */

           //lock the mutex
           std::lock_guard<std::mutex> lock(mtx);

           int value = q.front();

           ROS_WARN("Finished delay from agent %d", value);

           //remove from the queue
           q.pop();
        }

        void brakeConsumer() {
            bool done = false;

            while(true) {
                //Aquire mutex
                std::unique_lock<std::mutex> lock(queueMutex);

                //Wait for notify, queue to not be empty
                conditionVar.wait(lock, [this] {return !idQ.empty(); });
                
                //Dont need the mutex for the next few steps
                lock.unlock();

                set_mode.call(brake_msg);
                done = false;

                while (!done) {
                    //Block thread for some time
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

                    //Remove an item from the queue
                    removeFromQueue(idQ, queueMutex);
                    
                    //Check if the queue is empty
                    done = isQueueEmpty(idQ, queueMutex);
                }

                //set the drone back to guided 
                set_mode.call(guided_msg);

            }
            
        }
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
    /* 
    Logic for collision avoidance: 

        if (this agent is moving)
            if (colliding agent is moving)
                lower priority stops
                higher priority increases altitude
            else
                this agent increases altitude
        else
            take no action

    if both agents are moving
    */
    double decode_start = getUnixTime();

    int sending_agent = msg->id;
    rcvd_rs.intervals[0].min = msg->x.min;
    rcvd_rs.intervals[0].max = msg->x.max;
    rcvd_rs.intervals[1].min = msg->y.min;
    rcvd_rs.intervals[1].max = msg->y.max;
    rcvd_rs.intervals[2].min = msg->z.min;
    rcvd_rs.intervals[2].max = msg->z.max;
    //rcvd_rs.disp();

    double decode_end = getUnixTime(); //compute decode time
    
    std_msgs::Float32 dc_time;
    dc_time.data = decode_end - decode_start;
    dc_pub.publish(dc_time);

    if (my_rs.intersects(rcvd_rs)) {


        ROS_WARN("COLLISION DETECTED with agent %d", sending_agent);

        if (collision_action_enabled) {
            set_mode.call(brake_msg);
        }

        // -- collision deconfliction: BEGIN-- // 
        /*

        bool im_moving = (abs(my_rs.maxWidth()) > 8.0); //See if "I" am moving
        bool they_move = (abs(my_rs.maxWidth()) > 8.0); //See if "they" are moving

        if (im_moving) {
            //ROS_INFO("IM MOVING"); 
            if (they_move) {
                //Lower priority breaks
                //Higher priority Increases alt
                if (sending_agent < SYSID_THISMAV) { //If the sending agent is of higher priority
                    if (collision_action_enabled == true) {
                        if ( !isInQueue(idQ, sending_agent, queueMutex) ) {
                            //Add the sending agent to the queue
                            addToQueue(idQ, sending_agent, queueMutex);

                            //wake up the consumer
                            conditionVar.notify_one();
                        }

                        // ROS_WARN("I AM BRAKING");
                        // set_mode.call(brake_msg); //Stop the vehicle
                        // collision_avoidance_taken = true;
                        // std::thread(&collisionChecker::callGuided, this).detach();
                    }
                } else {
                    //Increase my alt
                    ROS_WARN("MOVING TO AVOIDANCE ALT");
                }
                // if (collision_action_enabled == false ) {
                //     std::thread(&collisionChecker::callGuided, this).detach();
                // }
                
            } else { //sending agent is not moving
                //increase altitude
                ROS_WARN("MOVING TO AVOIDANCE ALT");
            }
        } //else: do nothing

        // -- collision deconfliction END -- //
        */

    }

    double intersect_end = getUnixTime();
    std_msgs::Float32 in_time;
    in_time.data = intersect_end - decode_end;
    in_pub.publish(in_time);
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

    nh.param<int>("SystemID", SYSID_THISMAV, 0);
    ROS_WARN("collisionChecker: Set SYSID_THISMAV to %d", SYSID_THISMAV);

    my_rs = hyperRectangle(intervals);
    rcvd_rs = hyperRectangle(intervals);

    collision_action_enabled = false;

    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    brake_msg.request.custom_mode = "BRAKE";
    guided_msg.request.custom_mode = "GUIDED";

    my_rs_subscriber = nh.subscribe("/my_reach_state", 5, &collisionChecker::my_rs_cb, this);
    rcvd_rs_subscriber = nh.subscribe("/recieved_rs", 30, &collisionChecker::rcvd_rs_cb, this);
    collision_action_subscriber = nh.subscribe("/collision_action", 5, &collisionChecker::collision_action, this);

    //brakeThread = std::thread(&collisionChecker::brakeConsumer, this);

    dc_pub = nh.advertise<std_msgs::Float32>("dc_time", 1, true);
    in_pub = nh.advertise<std_msgs::Float32>("in_time", 1, true);

}

int main (int argc, char **argv) {
    ros::init(argc, argv, "collision_checker");
    collisionChecker cc;
    ros::spin();
    return 0;
}