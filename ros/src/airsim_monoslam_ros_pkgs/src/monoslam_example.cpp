#include "monoslam_example.h"


MonoslamExample::MonoslamExample(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  has_started = false;
  
  initialize_ros();
  
  // Reset Sim
  //airsim_ros_pkgs::Reset reset_req;
  //reset_svr.call(reset_req);

  // Wait and Takeoff
  ros::Duration(3).sleep();
  airsim_ros_pkgs::Takeoff takeoff_req;
  
  takeoff_req.request.waitOnLastTask = true;
  
  takeoff_svr.call(takeoff_req);
  
  ROS_INFO_STREAM("[Monoslam] TAKEOFF CALL");

  first_time = true;
  has_started = true;
  reached_checkpoint = true;
}

void MonoslamExample::initialize_ros()
{
  // ROS Params
  double update_control_every_n_sec;
  nh_private_.getParam("update_control_every_n_sec", update_control_every_n_sec);

  // ROS Services

  // Set Local Position
  set_local_pos_svr = nh_.serviceClient<airsim_ros_pkgs::SetLocalPosition>("/airsim_node/local_position_goal/override");

  // Takeoff
  takeoff_svr = nh_.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/Drone0/takeoff");
    
  // Land
  land_svr = nh_.serviceClient<airsim_ros_pkgs::Land>("/airsim_node/Drone0/land");

  // Reset
  reset_svr = nh_.serviceClient<airsim_ros_pkgs::Land>("/airsim_node/reset");

  // ROS Subscribers
  reached_checkpoint_sub = nh_.subscribe("/airsim_node/Drone0/reached_goal", 10, &MonoslamExample::reached_checkpoint_cb, this );
    
  // ROS Timers
  update_control_cmd_timer_ = nh_private_.createTimer(ros::Duration(update_control_every_n_sec), &MonoslamExample::update_control_cmd_timer_cb, this);
}

void MonoslamExample::update_control_cmd_timer_cb(const ros::TimerEvent& event)
{
  if (has_started && reached_checkpoint && index<cmd_cant)
    {
      
      airsim_ros_pkgs::SetLocalPosition TargetPosition;
      
      TargetPosition.request.x = cmd_sequence[index][0];
      TargetPosition.request.y = cmd_sequence[index][1];
      TargetPosition.request.z = cmd_sequence[index][2];
      TargetPosition.request.yaw = cmd_sequence[index][3];
      
      set_local_pos_svr.call(TargetPosition);
      
      ROS_INFO_STREAM("[Monoslam] Set Target Position x=" << TargetPosition.request.x << " y=" << TargetPosition.request.y << " z=" << TargetPosition.request.z << " yaw=" << TargetPosition.request.yaw );

      first_time = false;
      index+=1;
      
    }else if(has_started && reached_checkpoint && index==cmd_cant){
  
    airsim_ros_pkgs::Land land_req;
  
    land_req.request.waitOnLastTask = true;
  
    land_svr.call(land_req);
    
  }

}

void MonoslamExample::reached_checkpoint_cb(const std_msgs::Bool& reached_checkpoint_msg)
{
  reached_checkpoint = reached_checkpoint_msg.data;
  ROS_INFO_STREAM("[Monoslam] Reached Checkpoint " << reached_checkpoint );  
}

