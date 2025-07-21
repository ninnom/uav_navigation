#ifndef TWIST_CONTROLLER_NODE_H
#define TWIST_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "rotors_control/common.h"
#include "rotors_control/twist_controller.h"

namespace rotors_control {

class TwistControllerNode {
public:
  TwistControllerNode();
  ~TwistControllerNode();

  void InitializeParams();
  void Publish();

private:
  TwistController twist_controller_;

  std::string namespace_;

  bool auto_step_size;
  double t_in_sec;
  double dt_in_sec;
  double dt_in_sec_auto;

  // subscribers and publishers
  ros::Subscriber clock_sub_;
  ros::Subscriber step_sub_;
  ros::Subscriber cmd_twist_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber gt_imu_sub_;
  ros::Subscriber motor_sub_;

  ros::Publisher motor_velocity_reference_pub_;

  //  std::deque<ros::Duration> command_waiting_times_;
  //  ros::Timer command_timer_;

  // callbacks
  void StepsizeCallback(const std_msgs::Float64MultiArray& step_msg);

  void ClockCallback(const rosgraph_msgs::Clock& clock);

  void ImuCallback(const sensor_msgs::Imu& imu);

  //  void TimedCommandCallback(const ros::TimerEvent& e);

  void CommandTwistCallback(const geometry_msgs::Twist& twist); // command twist

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometryPtr_msg); // self twist

  void MotorStatusCallback(const std_msgs::Bool& motor_status_msg);
};
}

#endif // TWIST_CONTROLLER_NODE_H
