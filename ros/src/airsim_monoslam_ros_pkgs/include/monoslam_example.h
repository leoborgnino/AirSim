#ifndef _MONOSLAM_EXAMPLE_H_
#define _MONOSLAM_EXAMPLE_H_

#include <ros/ros.h>
#include <math.h>
#include <airsim_ros_pkgs/SetLocalPosition.h>
#include <airsim_ros_pkgs/Reset.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/Land.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class MonoslamExample
{
 public:
  MonoslamExample(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  // Timer update checkpoints to PD controller
  void process_image_cmd_timer_cb(const ros::TimerEvent& event);

  // Subscriber Image Received
  void image_received_cb(const sensor_msgs::Image& reached_checkpointimage_received_msg);

  // Initialization function
  void initialize_ros();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer process_image_cmd_timer_;

  ros::Subscriber image_received_sub;

  bool has_started;
  bool first_time;
  
};

#endif /* _MONOSLAM_EXAMPLE_ */

