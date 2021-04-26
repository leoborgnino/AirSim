#include "planning_example.h"

PlanningExample::PlanningExample(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
  has_started = false;

  //leatherman::ReadBinvox("/home/ingenia/Escritorio/Ingenia_Planning/AirSim/ros/src/airsim_planning_ros_pkgs/scripts/map.binvox");
  //leatherman::VoxelGrid grid = leatherman::Grid();
 
  
  initialize_ros();
  
  // Reset Sim
  //airsim_ros_pkgs::Reset reset_req;
  //reset_svr.call(reset_req);

  // Wait and Takeoff
  ros::Duration(3).sleep();
  airsim_ros_pkgs::Takeoff takeoff_req;
  
  takeoff_req.request.waitOnLastTask = true;
  
  takeoff_svr.call(takeoff_req);
  
  ROS_INFO_STREAM("[Planning] TAKEOFF CALL");

  first_time = true;
  has_started = true;
  reached_checkpoint = true;
 }

void PlanningExample::initialize_ros()
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
    reached_checkpoint_sub = nh_.subscribe("/airsim_node/Drone0/reached_goal", 10, &PlanningExample::reached_checkpoint_cb, this );
    
    // ROS Timers
    update_control_cmd_timer_ = nh_private_.createTimer(ros::Duration(update_control_every_n_sec), &PlanningExample::update_control_cmd_timer_cb, this);

    // Binvox Read
    std::vector<Eigen::Vector3f> voxels;
    
    leatherman::getOccupiedVoxelsInBinvoxFile("/home/ingenia/Escritorio/Ingenia_Planning/AirSim/ros/src/airsim_planning_ros_pkgs/scripts/map.binvox",voxels);
  
    // Spawn Model
    std::cout << "Waiting Gazebo" << std::endl;
    ros::service::waitForService("gazebo/spawn_urdf_model");
    spawn_model_client = nh_.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_urdf_model");
    std::cout << "Done Waiting Gazebo" << std::endl;
    
    gazebo_msgs::SpawnModel spawn_model;
    spawn_model.request.model_name = "box1";
    
    // load urdf file
    std::string urdf_filename = std::string("/home/ingenia/Escritorio/Ingenia_Planning/AirSim/ros/src/airsim_planning_ros_pkgs/scripts/box.urdf");
    ROS_INFO("loading file: %s",urdf_filename.c_str());
    // read urdf / gazebo model xml from file
    TiXmlDocument xml_in(urdf_filename);
    xml_in.LoadFile();
    std::ostringstream stream;
    stream << xml_in;
    std::cout << stream.str() << std::endl;
    spawn_model.request.model_xml = stream.str(); // load xml file
    ROS_INFO("XML string: %s",stream.str().c_str());
    
    spawn_model.request.robot_namespace = "";
    spawn_model.request.reference_frame = "";
    
    // spawn 10 boxes
    for (int i=0;i<20000;i++)
      {
	std::ostringstream mn_stream;
	mn_stream << "box_" << i;
	spawn_model.request.model_name = mn_stream.str();
	geometry_msgs::Pose pose;
	pose.position.x = voxels[voxels.size()-1-i][0];
	pose.position.y = voxels[voxels.size()-1-i](1);
	pose.position.z = voxels[voxels.size()-1-i](2);
	std::cout << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
	pose.orientation.w = 1.0; pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
	spawn_model.request.initial_pose = pose;
	spawn_model_client.call(spawn_model);
	std::cout << "Spawn " << i << std::endl;
      }
    
}

void PlanningExample::update_control_cmd_timer_cb(const ros::TimerEvent& event)
{
  if (has_started && reached_checkpoint && index<cmd_cant)
    {
      
      airsim_ros_pkgs::SetLocalPosition TargetPosition;
      
      TargetPosition.request.x = cmd_sequence[index][0];
      TargetPosition.request.y = cmd_sequence[index][1];
      TargetPosition.request.z = cmd_sequence[index][2];
      TargetPosition.request.yaw = cmd_sequence[index][3];
      
      set_local_pos_svr.call(TargetPosition);
      
      ROS_INFO_STREAM("[Planning] Set Target Position x=" << TargetPosition.request.x << " y=" << TargetPosition.request.y << " z=" << TargetPosition.request.z << " yaw=" << TargetPosition.request.yaw );

      first_time = false;
      index+=1;
      
    }else if(has_started && reached_checkpoint && index==cmd_cant){
  
    airsim_ros_pkgs::Land land_req;
  
    land_req.request.waitOnLastTask = true;
  
    land_svr.call(land_req);
    
  }

}

void PlanningExample::reached_checkpoint_cb(const std_msgs::Bool& reached_checkpoint_msg)
{
  reached_checkpoint = reached_checkpoint_msg.data;
  ROS_INFO_STREAM("[Planning] Reached Checkpoint " << reached_checkpoint );  
}
