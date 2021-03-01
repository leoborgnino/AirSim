#include "control_gazebo_example.h"

ControlGazeboExample::ControlGazeboExample(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    initialize_ros();
}

void ControlGazeboExample::initialize_ros()
{

  z_pose = 0.0;
  
  nh_private_.getParam("update_control_every_n_sec", update_control_every_n_sec);
  
  // Set model state Service
  set_state_svr = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  
  // ROS timers
  update_state_timer_ = nh_private_.createTimer(ros::Duration(update_control_every_n_sec), &ControlGazeboExample::update_state_timer_cb, this);
  
}

void ControlGazeboExample::update_state_timer_cb(const ros::TimerEvent& event)
{

  geometry_msgs::Pose start_pose;
  start_pose.position.x = 0.0;
  start_pose.position.y = 0.0;
  start_pose.position.z = z_pose;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 1.0;
  
  geometry_msgs::Twist start_twist;
  start_twist.linear.x = 0.0;
  start_twist.linear.y = 0.0;
  start_twist.linear.z = 0.0;
  start_twist.angular.x = 0.0;
  start_twist.angular.y = 0.0;
  start_twist.angular.z = 0.0;
  
  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = (std::string) "iris_demo";
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = start_pose;
  modelstate.twist = start_twist;

  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state = modelstate;

  set_state_svr.call(setmodelstate);
  
  z_pose = z_pose + 0.01;
  if(z_pose > 5)
    z_pose = 0.0;

}
