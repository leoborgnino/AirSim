#ifndef _CONTROL_GAZEBO_EXAMPLE_H_
#define _CONTROL_GAZEBO_EXAMPLE_H_

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "common/common_utils/FileSystem.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <math_common.h>
#include <utils.h>

#include <gazebo_msgs/ModelStates.h> // Gazebo's Tests
#include <gazebo_msgs/SetModelState.h> // Gazebo's Tests
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

class ControlGazeboExample
{
public:
    ControlGazeboExample(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void update_state_timer_cb(const ros::TimerEvent& event);

    void initialize_ros();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Timer update_state_timer_;

    ros::ServiceClient set_state_svr;

    double update_control_every_n_sec;
    double z_pose;

};

#endif
