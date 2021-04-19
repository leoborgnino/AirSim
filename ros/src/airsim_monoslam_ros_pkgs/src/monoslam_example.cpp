#include "monoslam_example.h"

static const std::string OPENCV_WINDOW = "Image window";

MonoslamExample::MonoslamExample(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  has_started = false;
  
  initialize_ros();
  
  ROS_INFO_STREAM("[Monoslam] ROS Initialized");

  cv::namedWindow(OPENCV_WINDOW);

  first_time = true;
  has_started = true;
}

void MonoslamExample::initialize_ros()
{
  // ROS Params
  double process_image_every_n_sec;
  nh_private_.getParam("process_image_every_n_sec", process_image_every_n_sec);

  std::string vehicle_name;
  do {
    nh_private_.getParam("/vehicle_name", vehicle_name);
    ROS_INFO_STREAM("Waiting vehicle name");
  } while(vehicle_name == "");
  
  // ROS Subscribers
  image_received_sub = nh_.subscribe("/airsim_node/" + vehicle_name + "/front_center_custom/Scene", 10, &MonoslamExample::image_received_cb, this );
    
  // ROS Timers
  process_image_cmd_timer_ = nh_private_.createTimer(ros::Duration(process_image_every_n_sec), &MonoslamExample::process_image_cmd_timer_cb, this);
}

void MonoslamExample::process_image_cmd_timer_cb(const ros::TimerEvent& event)
{
  if ( has_started )
    {
      //ROS_INFO_STREAM("[Monoslam] Ping");
      first_time = false; 
    }
}

void MonoslamExample::image_received_cb(const sensor_msgs::Image& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
  // Draw an example circle on the video stream
  //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
  
  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);
  
  //ROS_INFO_STREAM("[Monoslam] Image Received");  
}
