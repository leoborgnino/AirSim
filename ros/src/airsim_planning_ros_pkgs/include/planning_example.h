#ifndef _PLANNING_EXAMPLE_H_
#define _PLANNING_EXAMPLE_H_

#include <ros/ros.h>
#include <math.h>
#include <airsim_ros_pkgs/SetLocalPosition.h>
#include <airsim_ros_pkgs/Reset.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/Land.h>
#include <binvox.h>
#include <std_msgs/Bool.h>
#include <math_common.h>
#include <utils.h>
#include <iostream>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/DeleteModel.h>
#include "geometry_msgs/Pose.h"
#include "std_srvs/Empty.h"
#include <sstream>
#include <tinyxml.h>

class PlanningExample
{
 public:
  PlanningExample(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  // Timer update checkpoints to PD controller
  void update_control_cmd_timer_cb(const ros::TimerEvent& event);

  // Subscriber Reached Checkpoint Callback
  void reached_checkpoint_cb(const std_msgs::Bool& reached_checkpoint_msg);

  // Initialization function
  void initialize_ros();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer update_control_cmd_timer_;

  ros::ServiceClient set_local_pos_svr;
  ros::ServiceClient takeoff_svr;
  ros::ServiceClient land_svr;
  ros::ServiceClient reset_svr;

  ros::Subscriber reached_checkpoint_sub;

  bool reached_checkpoint;
  bool has_started;
  bool first_time;
  int cmd_cant=4;
  double cmd_sequence[4][4]={{5.0,0.0,-5.0,0.0},{5.0,5.0,-10.0,0.0},{10.0,0.0,-10.0,0.0},{20.0,0.0,-5.0,0.0}};
  int index=0;

  // Spawn Model
  ros::ServiceClient spawn_model_client;
};

#endif /* _PLANNING_EXAMPLE_ */

