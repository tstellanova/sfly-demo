/*
 * mav_status.h
 *
 *  Created on: Jan 27, 2012
 *      Author: acmarkus
 */

#ifndef MAV_STATUS_H_
#define MAV_STATUS_H_

#include <ros/ros.h>

#include <asctec_hl_comm/mav_status.h>
#include <asctec_hl_comm/DoubleArrayStamped.h>
#include <vismaggps_fusion/Status.h>
#include <mav_status/Status.h>

class MavStatus
{
private:
  ros::Subscriber sub_fcu_status_;
  ros::Subscriber sub_vismaggps_status_;
  ros::Subscriber sub_ssdk_debug_;

  ros::Publisher pub_status_;
  mav_status::Status status_msg_;

  ros::Time last_update_fcu_;
  ros::Time last_update_vismaggps_;
  static const double WATCHDOG_TIMEOUT = 1;

  std::string ns_;

  bool watchdog(const ros::Time & time);
  void fcuCallback(const asctec_hl_comm::mav_statusConstPtr & msg);
  void ssdkCallback(const asctec_hl_comm::DoubleArrayStampedConstPtr & msg);
  void vismaggpsCallback(const vismaggps_fusion::StatusConstPtr & msg);


public:
  MavStatus();
  void sendStatus();
};

#endif /* MAV_STATUS_H_ */
