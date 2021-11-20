//
// pixhawk_xform.cpp
//
// This node flips the axes on the mocap system
// so the correct positional data can be streamed
// to the Pixhawk via mavros
//
// Created by Adam Plowcha
//


#include <pixhawk_xform.h>

int main(int argc, char** argv)
{

    std::string topic_name = "/bugstomper";

    if (argc >1)
    {
        topic_name = "/";
        topic_name += argv[1];
        ROS_INFO("Pixhawk_xform using topic %s.", topic_name.c_str());
    }

    topic_name += "/ned/pose_stamped";
 
    //initialize ros
    ros::init(argc, argv, "pixhawk_xform");
    ros::NodeHandle nh;

    ROS_INFO("Pixhawk_xform starting...");

    //Setup pubs/subs/clients
 
    xform_pub = nh.advertise<geometry_msgs::PoseStamped>("pixhawk_xform/pose_stamped", 10);
    status_pub = nh.advertise<std_msgs::String>("pixhawk_xform/status", 10);

    outgoing_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav/mavros/setpoint_position/local", 10);

	range_pub = nh.advertise<sensor_msgs::Range>("mavros/distance_sensor/rangefinder_sub",1);

    mocap_sub = nh.subscribe(topic_name, 100, callbackMocap);
    command_sub = nh.subscribe("pixhawk_xform/command", 100, callbackCommand);

    incoming_pos_adjust_sub = nh.subscribe("pixhawk_xform/incoming_pos_adjust", 100, callbackIncomingPosAdjust);

	angle = 90.0; //initial adjustment of nose heading

    //main processing loop
    while (nh.ok())
    {

        ros::spinOnce();

    }

}

void callbackIncomingPosAdjust(const geometry_msgs::PoseStampedConstPtr &msg)
{
	//This method takes the incoming position assignment and adjusts
	//it by applying a rotation that matches the room rotation
	//
    incoming_rel_pos_adjust = *msg;

 	//Now we adjust for the rotation of the space.
   	double newx, newy;
   	double th = -32.0;
   	th = deg_to_rad(th);

   	newx = (cos(th) * incoming_rel_pos_adjust.pose.position.x) + (sin(th) * incoming_rel_pos_adjust.pose.position.y);
   	newy = (-sin(th) * incoming_rel_pos_adjust.pose.position.x) + (cos(th) * incoming_rel_pos_adjust.pose.position.y);

   	incoming_rel_pos_adjust.pose.position.x = newx;
   	incoming_rel_pos_adjust.pose.position.y = newy;

    outgoing_pos_pub.publish(incoming_rel_pos_adjust);
}

void callbackMocap(const geometry_msgs::PoseStampedConstPtr &msg)
{
   mocap = *msg;
   xformed = *msg;
   double th = -32.0;

   xformed.pose.position.x = mocap.pose.position.y;
   xformed.pose.position.y = mocap.pose.position.x;
   xformed.pose.position.z = -mocap.pose.position.z;
   
   //Now we adjust for the rotation of the space.
   double newx, newy;
   th = deg_to_rad(th);

   newx = (cos(th) * xformed.pose.position.x) + (sin(th) * xformed.pose.position.y);
   newy = (-sin(th) * xformed.pose.position.x) + (cos(th) * xformed.pose.position.y);

   xformed.pose.position.x = newx;
   xformed.pose.position.y = newy;

   //create a temp pose to adjust quaternion stuff
   geometry_msgs::Pose p = getDegFromQuat(xformed.pose);

   p.orientation.z += angle;

   //convert back to radians
   xformed.pose.orientation.z = deg_to_rad(p.orientation.z);

   geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(xformed.pose.orientation.x, xformed.pose.orientation.y, xformed.pose.orientation.z);

   xformed.pose.orientation = q; 

   xform_pub.publish(xformed);

   //range finder stuff
   sensor_msgs::Range r;
   r.min_range = 0.25;
   r.max_range = 3.0;
   r.range = xformed.pose.position.z;
   range_pub.publish(r);

}

geometry_msgs::Pose getDegFromQuat(geometry_msgs::Pose p)
{
    geometry_msgs::Pose retP;

    //Change this later after we figure out the NED/world frame stuff
    retP.position.x = p.position.x;
    retP.position.y = p.position.y;
    retP.position.z = p.position.z;

    tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    retP.orientation.x = rad_to_deg(pitch);
    retP.orientation.y = rad_to_deg(roll);
    retP.orientation.z = rad_to_deg(yaw);

    return retP;
}


void callbackCommand(const std_msgs::StringConstPtr &msg)
{
    //Receive commands from the gui and process as
    //needed
    std::string cmd = msg->data.c_str();

    ROS_INFO_STREAM("Command received: " << msg->data);

    if (cmd == "s") //s = show current angular bias
    {
		//publish to the status topic
		status.data = std::to_string(angle);
		status_pub.publish(status);
		ROS_INFO_STREAM("Angular bias: " << angle);
	}	
	else
	{
		//convert to double and set new angular bias
		angle = std::stod(cmd);

		//pub to the status topic to provide positive 
		//feeback for variable change
		status.data = std::to_string(angle);
		status_pub.publish(status);
		ROS_INFO_STREAM("Angular bias: " << angle);
	}
}
