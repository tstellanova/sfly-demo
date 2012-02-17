/*
 * joy.h
 *
 *  Created on: Feb 17, 2012
 *      Author: acmarkus
 */

#ifndef JOY_H_
#define JOY_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include "change_detector.h"

struct AxisAssignment
{
  int x;
  int y;
  int z;
  int yaw;
};

struct ButtonAssignment
{
  int takeoff;
  int ctrl_enable;
  int ptam_reset;
  int ptam_space;
  int ptam_auto_init;
  int filter_init;
};

class Joy
{
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

private:
  ros::Publisher ctrl_pub_;
  ros::Publisher ptam_cmd_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber pose_sub_;

  AxisAssignment axes_;
  ButtonAssignment buttons_;

  bool first_joy_callback_;

  void joyCallback(const sensor_msgs::JoyConstPtr & msg);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr & msg);

  void takeoffCallback(int val);
  void filterInitCallback(int val);
  void ptamSpaceCallback(int val);
  void ptamResetCallback(int val);
  void ptamAutoInitCallback(int val);
public:
  Joy();
  ChangeDetector<std::vector<int> > button_handler_;
};

#endif /* JOY_H_ */
