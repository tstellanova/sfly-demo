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

  ptam_cmd_pub_ = nh.advertise<std_msgs::String> ("vslam/key_pressed", 10);

  joy_sub_ = nh.subscribe("joy", 10, &Joy::joyCallback, this);
  pose_sub_ = nh.subscribe("pose", 10, &Joy::poseCallback, this);

  // defaults for Logitech Wireless Gamepad F710
  pnh.param("channel_x_", axes_.x, 3);
  pnh.param("channel_y_", axes_.y, 2);
  pnh.param("channel_z_", axes_.z, 1);
  pnh.param("channel_yaw_", axes_.yaw, 0);
  pnh.param("channel_takeoff_", buttons_.takeoff, 9);
  pnh.param("channel_ctrl_enable_", buttons_.ctrl_enable, 4);
  pnh.param("channel_ptam_reset_", buttons_.ptam_reset, 2);
  pnh.param("channel_ptam_space_", buttons_.ptam_space, 3);
  pnh.param("channel_ptam_auto_init_", buttons_.ptam_auto_init, 1);
  pnh.param("channel_filter_init_", buttons_.filter_init, 0);

  button_handler_.registerCallback(buttons_.takeoff, &Joy::takeoffCallback, this);
  button_handler_.registerCallback(buttons_.filter_init, &Joy::filterInitCallback, this);
  button_handler_.registerCallback(buttons_.ptam_space, &Joy::ptamSpaceCallback, this);
  button_handler_.registerCallback(buttons_.ptam_reset, &Joy::ptamResetCallback, this);
  button_handler_.registerCallback(buttons_.ptam_auto_init, &Joy::ptamAutoInitCallback, this);

}

void Joy::joyCallback(const sensor_msgs::JoyConstPtr & msg)
{

  if (first_joy_callback_)
  {
    button_handler_.init(msg->buttons);
    first_joy_callback_ = false;
    return;
  }

  button_handler_.update(msg->buttons);
}

void Joy::poseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
  ;
}

void Joy::takeoffCallback(int val)
{
  std::cout << "takeoff: " << val << std::endl;
}

void Joy::filterInitCallback(int val)
{
  std::cout << "filterinit: " << val << std::endl;
  if(val == 1){
    system("rosrun dynamic_reconfigure dynparam set vismaggps filter_init true --timeout=2");
  }
}

void Joy::ptamSpaceCallback(int val)
{
  std_msgs::StringPtr msg(new std_msgs::String);
  if (val == 1)
  {
    msg->data = "Space";
    ptam_cmd_pub_.publish(msg);
    ROS_INFO("Sent \"Space\" to ptam");
  }
}

void Joy::ptamResetCallback(int val)
{
  std_msgs::StringPtr msg(new std_msgs::String);
  if (val == 1)
  {
    msg->data = "r";
    ptam_cmd_pub_.publish(msg);
    ROS_INFO("Sent \"r\" to ptam");
  }
}

void Joy::ptamAutoInitCallback(int val)
{
  std_msgs::StringPtr msg(new std_msgs::String);
  if (val == 1)
  {
    msg->data = "a";
    ptam_cmd_pub_.publish(msg);
    ROS_INFO("Sent \"a\" to ptam");
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mav_joy");
  Joy joy;

  ros::spin();

  return 0;
}
