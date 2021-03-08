#ifndef _PLANNING_EXAMPLE_H_
#define _PLANNING_EXAMPLE_H_

#include <ros/ros.h>
#include <math.h>
#include <airsim_ros_pkgs/SetLocalPosition.h>
#include <airsim_ros_pkgs/Reset.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/Land.h>
#include <math_common.h>
#include <utils.h>

class PlanningExample
{
 public:
  PlanningExample(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  void update_control_cmd_timer_cb(const ros::TimerEvent& event);

  void initialize_ros();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer update_control_cmd_timer_;

  ros::ServiceClient set_local_pos_svr;
  ros::ServiceClient takeoff_svr;
  ros::ServiceClient land_svr;
  ros::ServiceClient reset_svr;

  bool has_started;
  bool first_time;
};

#endif /* _PLANNING_EXAMPLE_ */
