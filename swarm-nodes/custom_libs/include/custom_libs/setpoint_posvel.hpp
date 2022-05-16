
#ifndef SETPOINT_POSVEL_H
#define SETPOINT_POSVEL_H

#include <ros/ros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <mavros/utils.h>
#include <mavros/mavros_uas.h>
#include <mavros/mavros_plugin.h>
//#include <mavros/plugin_filter.h>
#include <mavros/setpoint_mixin.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>
//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>

namespace mavros {

namespace std_plugis {

//using namespace std::placeholders;      // NOLINT
using mavlink::common::MAV_FRAME;

MAV_FRAME mav_frame;

class setpoint_posvel: public plugin::PluginBase, public plugin::SetPositionTargetLocalNEDMixin<setpoint_posvel>
{



//friend class SetPositionTargetLocalNEDMixin;
friend class plugin::SetPositionTargetLocalNEDMixin<setpoint_posvel>;




public:
//	MAV_FRAME mav_frame;
//      ros::NodeHandle nh;

//      ros::Subscriber vel_sub = nh.subscribe("~/cmd_posvel", 100, std::bind(&setpoint_posvel_plg::vel_cb,this, _1));
//       ros::Subscriber vel_sub = nh.subscribe("~/cmd_posvel", 100, vel_cb);


        //friend class plugin::SetPositionTargetLocalNEDMixin<setpoint_posvel>;

//	friend class SetPositionTargetLocalNEDMixin;

	void send_setpoint_pos_velocity( uint32_t time_boot_ns, const Eigen::Vector3d & vel_enu, const double yaw_rate) ;

};

}

}



#endif
