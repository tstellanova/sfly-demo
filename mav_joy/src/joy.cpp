/*
 * joy.cpp
 *
 *  Created on: Feb 17, 2012
 *      Author: acmarkus
 */

#include "joy.h"
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>

Joy::Joy() :
  first_joy_callback_(true)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ptam_cmd_pub_ = nh_.advertise<std_msgs::String> ("vslam/key_pressed", 10);

  ctrl_pub_ = nh_.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control_body", 10);

  mav_ctrl_.header.seq = 1;
  mav_ctrl_.type = asctec_hl_comm::mav_ctrl::velocity;
  mav_ctrl_.v_max_xy = 0;
  mav_ctrl_.v_max_z = 0;
  mav_ctrl_.x = 0;
  mav_ctrl_.y = 0;
  mav_ctrl_.z = 0;
  mav_ctrl_.yaw = 0;

  joy_sub_ = nh_.subscribe("joy", 10, &Joy::joyCallback, this);
  //  pose_sub_ = nh_.subscribe("fcu/current_pose", 10, &Joy::poseCallback, this);

  pnh.param("v_max_xy", v_max_.xy, 1.0); // m/s
  pnh.param("v_max_z", v_max_.z, 1.0); // m/s
  pnh.param("v_max_yaw", v_max_.yaw, 45.0); // deg/s
  v_max_.yaw *= (M_PI / 180.0);

  // defaults for Logitech Wireless Gamepad F710
  pnh.param("channel_x_", axes_.x, 3);
  pnh.param("channel_y_", axes_.y, 2);
  pnh.param("channel_z_", axes_.z, 1);
  pnh.param("channel_yaw_", axes_.yaw, 0);
  pnh.param("channel_takeoff_", buttons_.takeoff, 9);
  pnh.param("channel_ctrl_enable_", buttons_.ctrl_enable, 4);
  pnh.param("channel_ctrl_enable_", buttons_.function_enable, 5);
  pnh.param("channel_ptam_reset_", buttons_.ptam_reset, 2);
  pnh.param("channel_ptam_space_", buttons_.ptam_space, 3);
  pnh.param("channel_ptam_auto_init_", buttons_.ptam_auto_init, 1);
  pnh.param("channel_filter_init_", buttons_.filter_init, 0);

  pnh.param("direction_x", axes_direction_.x, 1);
  pnh.param("direction_y", axes_direction_.y, 1);
  pnh.param("direction_z", axes_direction_.z, 1);
  pnh.param("direction_yaw", axes_direction_.yaw, 1);

  button_handler_.registerCallback(buttons_.takeoff, &Joy::takeoffCallback, this);
  button_handler_.registerCallback(buttons_.filter_init, &Joy::filterInitCallback, this);
  button_handler_.registerCallback(buttons_.ptam_space, &Joy::ptamSpaceCallback, this);
  button_handler_.registerCallback(buttons_.ptam_reset, &Joy::ptamResetCallback, this);
  button_handler_.registerCallback(buttons_.ptam_auto_init, &Joy::ptamAutoInitCallback, this);
  button_handler_.registerCallback(buttons_.ctrl_enable, &Joy::ctrlEnableCallback, this);

  namespace_ = nh.getNamespace();
  if (namespace_ != "/")
  {
    while (namespace_.at(0) == '/')
      namespace_.erase(0, 1);
    namespace_ = "/" + namespace_;
  }
}

bool Joy::controlEnabled()
{
  return current_joy_.buttons[buttons_.ctrl_enable] == 1;
}

bool Joy::functionsEnabled()
{
  return current_joy_.buttons[buttons_.function_enable] == 1;
}

void Joy::joyCallback(const sensor_msgs::JoyConstPtr & msg)
{
  current_joy_ = *msg;

  if (first_joy_callback_)
  {
    button_handler_.init(msg->buttons);
    first_joy_callback_ = false;
    return;
  }
  button_handler_.update(msg->buttons);

  if (controlEnabled())
  {
    sendMavCommand(msg);
    watchdog_timer_ = nh_.createTimer(ros::Duration(5), &Joy::watchdog, this, true, true);
  }
}

bool Joy::sendMavCommand(const sensor_msgs::JoyConstPtr & msg)
{

  int size = msg->axes.size();
  if (axes_.x >= size || axes_.y >= size || axes_.z >= size || axes_.yaw >= size)
  {
    ROS_ERROR("Axis index exceeds axes size: x:%d y:%d z:%d yaw:%d ", axes_.x, axes_.y, axes_.z, axes_.yaw);
    return false;
  }

  //  const geometry_msgs::Quaternion & q = current_pose_.pose.orientation;
  //  // get only yaw from the quaternion and rotate vx, vy from body to world coordinates, most likely not the most efficient way ...
  //  float yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  //  float s_yaw = sin(yaw);
  //  float c_yaw = cos(yaw);
  //  float vx_b = msg->axes[axes_.x];
  //  float vy_b = msg->axes[axes_.y];
  //
  //  mav_ctrl_.header.seq++;
  //  mav_ctrl_.header.stamp = msg->header.stamp;
  //  mav_ctrl_.type = asctec_hl_comm::mav_ctrl::velocity;
  //  mav_ctrl_.v_max_xy = v_max_.xy;
  //  mav_ctrl_.v_max_z = v_max_.z;
  //  mav_ctrl_.x = (c_yaw * vx_b - s_yaw * vy_b) * v_max_.xy * axes_direction_.x;
  //  mav_ctrl_.y = (s_yaw * vx_b + c_yaw * vy_b) * v_max_.xy * axes_direction_.y;
  //  mav_ctrl_.z = msg->axes[axes_.z] * v_max_.z * axes_direction_.z;
  //  mav_ctrl_.yaw = msg->axes[axes_.yaw] * v_max_.yaw * axes_direction_.yaw;

  mav_ctrl_.header.seq++;
  mav_ctrl_.header.stamp = ros::Time::now();//msg->header.stamp;
  mav_ctrl_.type = asctec_hl_comm::mav_ctrl::velocity_body;
  mav_ctrl_.v_max_xy = v_max_.xy;
  mav_ctrl_.v_max_z = v_max_.z;
  mav_ctrl_.x = msg->axes[axes_.x] * v_max_.xy * axes_direction_.x;
  mav_ctrl_.y = msg->axes[axes_.y] * v_max_.xy * axes_direction_.y;
  mav_ctrl_.z = msg->axes[axes_.z] * v_max_.z * axes_direction_.z;
  mav_ctrl_.yaw = msg->axes[axes_.yaw] * v_max_.yaw * axes_direction_.yaw;

  ctrl_pub_.publish(mav_ctrl_);

  return true;
}

void Joy::stopMav()
{
  mav_ctrl_.type = asctec_hl_comm::mav_ctrl::velocity;
  mav_ctrl_.v_max_xy = 0;
  mav_ctrl_.v_max_z = 0;
  mav_ctrl_.x = 0;
  mav_ctrl_.y = 0;
  mav_ctrl_.z = 0;
  mav_ctrl_.yaw = 0;

  ros::Rate r(10);
  for (int i = 0; i < 10; i++)
  {
    mav_ctrl_.header.seq++;
    mav_ctrl_.header.stamp = ros::Time::now();
    ctrl_pub_.publish(mav_ctrl_);
    r.sleep();
  }
}

void Joy::watchdog(const ros::TimerEvent & e)
{
  stopMav();
  ROS_WARN("Joystick watchdog triggered");
}

void Joy::ctrlEnableCallback(int val)
{
  if (val == 0)
  {
    watchdog_timer_.stop(); //everything went well, so we can stop the timer
    stopMav();
  }
}

void Joy::poseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
  current_pose_ = *msg;
}

void Joy::takeoffCallback(int val)
{
  std::cout << "takeoff: " << val << std::endl;
}

void Joy::filterInitCallback(int val)
{
  if (functionsEnabled())
  {
    if (val == 1)
    {
      setDynParam("vismaggps_fusion init_filter true");
    }
  }
}

void Joy::ptamSpaceCallback(int val)
{
  if (functionsEnabled())
  {
    std_msgs::StringPtr msg(new std_msgs::String);
    if (val == 1)
    {
      msg->data = "Space";
      ptam_cmd_pub_.publish(msg);
      ROS_INFO("Sent \"Space\" to ptam");
    }
  }
}

void Joy::ptamResetCallback(int val)
{
  if (functionsEnabled())
  {
    std_msgs::StringPtr msg(new std_msgs::String);
    if (val == 1)
    {
      msg->data = "r";
      ptam_cmd_pub_.publish(msg);
      ROS_INFO("Sent \"r\" to ptam");
    }
  }
}

void Joy::ptamAutoInitCallback(int val)
{
  if (val == 1)
  {
    ROS_WARN("Autoinit not implemented yet ;)");
    // dynreconf call ...
  };
}

bool Joy::setDynParam(const std::string & param_string)
{
  int ret = system(std::string("rosrun dynamic_reconfigure dynparam set --timeout=2 " + param_string).c_str());

  ROS_INFO_COND(ret == 0, "dynamic reconfigure call for %s succeeded", param_string.c_str());
  ROS_WARN_COND(ret != 0, "dynamic reconfigure call for %s failed", param_string.c_str());

  return ret == 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mav_joy");
  Joy joy;

  ros::spin();

  return 0;
}
