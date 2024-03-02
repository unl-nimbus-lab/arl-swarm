#include "state_provider/state_provider.h"

StateProvider::StateProvider() {


    nh.param<int>("SystemID", SYSID_THISMAV, 1);
    nh.param<double>("global_origin_lat", global_origin.lat, 40.846740);
    nh.param<double>("global_origin_lon", global_origin.lon, -96.471819);

    state_pub = nh.advertise<geometry_msgs::AccelStamped>("/agent_state", 10, true);

    pos_sub = nh.subscribe("/mavros/global_position/swarm", 10, &StateProvider::posCallback, this);
    vel_sub = nh.subscribe("/mavros/global_position/local", 10, &StateProvider::velCallback, this);

    state_pub_timer = nh.createTimer(ros::Duration(0.04), &StateProvider::publishState, this);

}

double sign(double input) {
    //Deterine the sign of a double
    if (input < 0) {
        return -1.0;
    } else {
        return 1.0;
    }
}

void StateProvider::posCallback(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
    if (msg->system_id == SYSID_THISMAV) {//Check if this message is for this agent
        //This message is for this agent
        double lat = msg->latitude;
        double lon = msg->longitude;

        //Assumption: we are working in the northern hemisphere
        double y_sign = sign(lat - global_origin.lat);
        double x_sign = sign(lon - global_origin.lon);

        agent_state.y = converter.gps_to_meters(global_origin.lat, global_origin.lon, lat, global_origin.lon);
        agent_state.y = y_sign * agent_state.y;

        agent_state.x = converter.gps_to_meters(global_origin.lat, global_origin.lon, global_origin.lat, lon);
        agent_state.x = x_sign * agent_state.x;

        agent_state.z = msg->altitude;
    }
}

void StateProvider::velCallback(const nav_msgs::Odometry::ConstPtr& msg) { //This is assuming we are only getting velocity from the host agent
    agent_state.vx = msg->twist.twist.linear.x;
    agent_state.vy = msg->twist.twist.linear.y;
    agent_state.vz = -1.0 * msg->twist.twist.linear.z;
}

void StateProvider::publishState(const ros::TimerEvent& event) {
    //Encode the agent's position into the linear part of the accel message
    state_msg.accel.linear.x = agent_state.x;
    state_msg.accel.linear.y = agent_state.y;
    state_msg.accel.linear.z = agent_state.z;

    //Encode the agent's velocity into the angular part of the accel message
    state_msg.accel.angular.x = agent_state.vx;
    state_msg.accel.angular.y = agent_state.vy;
    state_msg.accel.angular.z = agent_state.vz;

    state_msg.header.stamp = ros::Time::now();

    state_pub.publish(state_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_provider");
    StateProvider state_provider;
    ros::spin();
}


//This node subsribes to the agent's gps location and converts it to a local XY frame
//centered at a pre-determined orgin (RMF for instance)

// struct { //Global origin is stored here, should not change
//     float lat = 40.846740;
//     float lon = -96.471819;
// } global_origin;

// struct { //Agent's state position is store here, eventually published
//     float x = 0.0;
//     float y = 0.0;
//     float z = 0.0;
//     float vx = 0.0;
//     float vy = 0.0;
//     float vz = 0.0;
// } agent_state;

// int SYSID_THISMAV = 1; //This agent's ID

// geo_converter converter;

// double sign(double input);

// void pos_callback(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
//     if (msg->system_id == SYSID_THISMAV) {//Check if this message is for this agent
//         //This message is for this agent
//         double lat = msg->latitude;
//         double lon = msg->longitude;

//         //Assumption: we are working in the northern hemisphere
//         double y_sign = sign(lat - global_origin.lat);
//         double x_sign = sign(lon - global_origin.lon);

//         agent_state.y = converter.gps_to_meters(global_origin.lat, global_origin.lon, lat, global_origin.lon);
//         agent_state.y = y_sign * agent_state.y;

//         agent_state.x = converter.gps_to_meters(global_origin.lat, global_origin.lon, global_origin.lat, lon);
//         agent_state.x = x_sign * agent_state.x;

//         agent_state.z = msg->altitude;
//     }
// }

// void vel_callback(const nav_msgs::Odometry::ConstPtr& msg) { //This is assuming we are only getting velocity from the host agent
//     agent_state.vx = msg->twist.twist.linear.x;
//     agent_state.vy = msg->twist.twist.linear.y;
//     agent_state.vz = -1.0 * msg->twist.twist.linear.z;
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "state_provider");
//     ros::NodeHandle nh;

//     //ROS subscribers
//     ros::Subscriber pos_sub = nh.subscribe("/agent_1/mavros/global_position/swarm", 10, pos_callback);
//     ros::Subscriber vel_sub = nh.subscribe("/agent_1/mavros/global_position/local", 10, vel_callback);

//     //ROS publishers
//     ros::Publisher state_pub = nh.advertise<geometry_msgs::AccelStamped>("/agent_state", 10, true);

//     ros::Rate loop_rate(25);
//     geometry_msgs::AccelStamped state_message; //reuse the accelStamped message because I don't want to make a new one

//     while (ros::ok()) {
//         //Publish our agent's state

//         //Encode the agent's position into the linear part of the accel message
//         state_message.accel.linear.x = agent_state.x;
//         state_message.accel.linear.y = agent_state.y;
//         state_message.accel.linear.z = agent_state.z;

//         //Encode the agent's velocity into the angular part of the accel message
//         state_message.accel.angular.x = agent_state.vx;
//         state_message.accel.angular.y = agent_state.vy;
//         state_message.accel.angular.z = agent_state.vz;

//         state_message.header.stamp = ros::Time::now();

//         state_pub.publish(state_message);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }

// double sign(double input) {
//     //Deterine the sign of a double
//     if (input < 0) {
//         return -1.0;
//     } else {
//         return 1.0;
//     }
// }
