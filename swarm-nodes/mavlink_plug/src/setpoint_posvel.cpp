// This package creates a function to generate a mavlink msg to set both position and velocity in the fcu
// TwistStamped msg is slightly violated to pass both position and velocity 

/*#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

//#include <mavros/mavros_uas.h>
//#include <mavros/plugin.h>
//#include "mavros/plugin_filter.hpp"
#include <mavros/setpoint_mixin.h>
#include <geometry_msgs/TwistStamped.h>
//#include "geometry_msgs/msg/twist_stamped.hpp"
//#include "geometry_msgs/msg/twist.hpp"

#define ROS_NODE_NAME "setpoint_posvel"

using namespace mavros;
//using mavros::setpoint_mixin ;

void posvel_callback(const geometry_msgs::TwistStamped::ConstPtr& req)
{

	// TwistStamp angular is used to pass velocity
	Eigen::Vector3d pos_val;
	Eigen::Vector3d vel_val;

	auto stamp = req->header.stamp ;
	tf2::fromMsg(req->twist.linear, pos_val);
	tf2::fromMsg(req->twist.angular, vel_val);

	uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);

	setpoint_mixin::set_position_target_local_ned(
     		stamp.nsec/10,
       		utils::enum_value(mav_frame),
      		ignore_all_except_v_xyz_yr,
      		Eigen::Vector3d::Zero(),
      		vel_val,
      		Eigen::Vector3d::Zero(),
      		0.0, 0.0);


}


int main( int argc, char** argv )
{
        ros::init(argc, argv, ROS_NODE_NAME); //Initialize the node

        ros::NodeHandle stpnt_posvel_hl;
        ros::Subscriber posvel_callback_handle = stpnt_posvel_hl.subscribe("/swarm/setpoint_posvel", 100, posvel_callback);
 //       ros::Subscriber local_pos_callback = post_hrl_hl.subscribe("/mavros/global_position/local", 100, localPosCallback);



        ros::spin();
        return 0;
}
*/
//#define BOOST_BIND_NO_PLACEHOLDERS


#include <tf2_eigen/tf2_eigen.h>
#include <ros/ros.h>
#include <mavros/mavros_uas.h>
#include <mavros/mavros_plugin.h>
//#include <mavros/plugin_filter.h>
#include <mavros/setpoint_mixin.h>
#include <geometry_msgs/TwistStamped.h>
//#include <rcpputils/asserts.h>
//#include <rclcpp/rclcpp.h>


namespace mavros {

namespace std_plugins {
//namespace plugin {

using namespace std::placeholders;      // NOLINT
using mavlink::common::MAV_FRAME;

class setpoint_posvel_plg: public plugin::PluginBase, public plugin::SetPositionTargetLocalNEDMixin<setpoint_posvel_plg>
{

//mav_frame = MAV_FRAME::BODY_NED ;

public:


	ros::NodeHandle nh;
	setpoint_posvel_plg() : PluginBase(),
		nh("~")
	{ }



	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		vel_sub = sp_nh.subscribe("pos_vel", 10, &SetpointVelocityPlugin::posvel_cb, this);

		//cmd_vel usually is the topic used for velocity control in many controllers / planners
	//	vel_sub = sp_nh.subscribe("cmd_vel", 10, &SetpointVelocityPlugin::vel_cb, this);
	//	vel_unstamped_sub = sp_nh.subscribe("cmd_vel_unstamped", 10, &SetpointVelocityPlugin::vel_unstamped_cb, this);
	//	mav_frame_srv = sp_nh.advertiseService("mav_frame", &SetpointVelocityPlugin::set_mav_frame_cb, this);

		// mav_frame
	//	std::string mav_frame_str;
	//	if (!sp_nh.getParam("mav_frame", mav_frame_str)) {
	//		mav_frame = MAV_FRAME::LOCAL_NED;
	//	} else {
	//		mav_frame = utils::mav_frame_from_str(mav_frame_str);
	//	}

	}

	Subscriptions get_subscriptions() override
	{
		return { /* Rx disabled */ };
	}




//	  explicit setpoint_posvel_plg(plugin::UASPtr uas_)
//  		: Plugin(uas_, "setpoint_velocity")
//{
//	MAV_FRAME mav_frame;
//	ros::NodeHandle nh;

//	ros::Subscriber vel_sub = nh.subscribe("~/cmd_posvel", 100, std::bind(&setpoint_posvel_plg::vel_cb,this, _1));
//	 ros::Subscriber vel_sub = nh.subscribe("~/cmd_posvel", 100, vel_cb);

	MAV_FRAME mav_frame ;
	friend class plugin::SetPositionTargetLocalNEDMixin<setpoint_posvel_plg>;
//	void send_setpoint_pos_velocity();
//	void send_setpoint_velocity(uint32_t time_boot_ns, const Eigen::Vector3d & vel_enu, const double yaw_rate);
//          void vel_cb(const geometry_msgs::TwistStamped::ConstPtr req);


//	auto vel_sub = node->create_subscription<geometry_msgs::TwistStamped>(
//    	"~/cmd_posvel", std::bind(&setpoint_posvel_plg::vel_cb, this));
/*(
        Subscriptions get_subscriptions() override {
                return {
                        make_handler(&setpoint_posvel::handle_heartbeat)
                };
        }
*/

//	}
friend class SetPositionTargetLocalNEDMixin;

void send_setpoint_velocity(
    uint32_t time_boot_ns, const Eigen::Vector3d & vel_enu,
    const double yaw_rate)
  {
    /**
     * Documentation start from bit 1 instead 0;
     * Ignore position and accel vectors, yaw.
     */
    uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);

	mav_frame = MAV_FRAME::BODY_NED ;
                set_position_target_local_ned(
                        time_boot_ns,
                        utils::enum_value(mav_frame),
                        ignore_all_except_v_xyz_yr,
                        Eigen::Vector3d::Zero(),
                        ftf::transform_frame_enu_ned(vel_enu),
                        Eigen::Vector3d::Zero(),
                        0.0, ftf::transform_frame_ned_enu(Eigen::Vector3d(0.0, 0.0, yaw_rate)).z() );



  }


void posvel_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		Eigen::Vector3d vel_enu;

		tf::vectorMsgToEigen(req->twist.linear, vel_enu);
		ROS_INFO("Inside Vel Pos Callback");
		//send_setpoint_velocity(req->header.stamp, vel_enu,
					req->twist.angular.z);
	}


private :
//	ros::NodeHandle nh;



/*private:
	friend class plugin::SetPositionTargetLocalNEDMixin<setpoint_posvel_plg>;
        ros::NodeHandle nh;
	MAV_FRAME mav_frame;

/*
	void send_setpoint_pos_velocity(uint32_t time_boot_ns, const Eigen::Vector3d & vel_enu, const double yaw_rate)
	{

		mav_frame == MAV_FRAME::BODY_NED ;

		uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);


		set_position_target_local_ned(
			time_boot_ns,
      			utils::enum_value(mav_frame),
      			ignore_all_except_v_xyz_yr,
      			Eigen::Vector3d::Zero(),
      			ftf::transform_frame_enu_ned(vel_enu),
      			Eigen::Vector3d::Zero(),
      			0.0, ftf::transform_frame_ned_enu(Eigen::Vector3d(0.0, 0.0, yaw_rate)).z() );


	}
*/
/*(	  void vel_cb(const geometry_msgs::TwistStamped::ConstPtr req)
	{
	    	Eigen::Vector3d vel_enu;

    		tf2::fromMsg(req->twist.linear, vel_enu);
    		send_setpoint_velocity(
    	  	req->header.stamp.nsec/100, vel_enu,
      		0.0);
  	}
*/
};

}
}


/*void send_setpoint_pos_velocity(uint32_t time_boot_ns, const Eigen::Vector3d & vel_enu, const double yaw_rate)
        {

//		friend class plugin::SetPositionTargetLocalNEDMixin<setpoint_posvel_plg>;
                mav_frame = MAV_FRAME::BODY_NED ;

                uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);


                set_position_target_local_ned(
                        time_boot_ns,
                        utils::enum_value(mav_frame),
                        ignore_all_except_v_xyz_yr,
                        Eigen::Vector3d::Zero(),
                        ftf::transform_frame_enu_ned(vel_enu),
                        Eigen::Vector3d::Zero(),
                        0.0, ftf::transform_frame_ned_enu(Eigen::Vector3d(0.0, 0.0, yaw_rate)).z() );


        }
*/
/*void vel_cb(const geometry_msgs::TwistStamped::ConstPtr req)
        {
                Eigen::Vector3d vel_enu;

                tf2::fromMsg(req->twist.linear, vel_enu);
                send_setpoint_velocity(
                req->header.stamp.nsec/100, vel_enu,
                0.0);
        };


*/



//}       // namespace std_plugins
       // namespace mavros





//#include <mavros/mavros_plugin_register_macro.hpp>  
//MAVROS_PLUGIN_REGISTER(mavros::std_plugins::setpoint_posvel_plg)

//#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::setpoint_posvel_plg, mavros::plugin::PluginBase)


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::setpoint_posvel_plg, mavros::plugin::PluginBase)
