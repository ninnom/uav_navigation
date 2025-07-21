
#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "twist_controller_node.h"
#include <rotors_control/parameters_ros.h>

namespace rotors_control{

TwistControllerNode::TwistControllerNode():
  t_in_sec(0.0),
  dt_in_sec(0.001),
  dt_in_sec_auto(0.001),
  auto_step_size(false) {

InitializeParams();

ros::NodeHandle nh;

   // Subscriber for action twist message
  cmd_twist_sub_= nh.subscribe("cmd_vel", 1, &TwistControllerNode::CommandTwistCallback, this);


  clock_sub_ = nh.subscribe("/clock", 1, &TwistControllerNode::ClockCallback, this);

  motor_sub_ = nh.subscribe("motor_status",1, &TwistControllerNode::MotorStatusCallback, this);

  step_sub_ = nh.subscribe("sim_update_event", 1, &TwistControllerNode::StepsizeCallback, this);

  gt_imu_sub_ = nh.subscribe(kDefaultImuTopic, 1, &TwistControllerNode::ImuCallback, this);

  odom_sub_ = nh.subscribe(kDefaultOdometryTopic, 1, &TwistControllerNode::OdometryCallback, this);

  // Publisher
  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(kDefaultCommandMotorSpeedTopic, 1);

// command_timer_ = nh.createTimer(ros::Duration(0), &TwistControllerNode::TimedCommandCallback, this, true, false);
}

TwistControllerNode::~TwistControllerNode() { }

void TwistControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // read controller parameters
  twist_controller_.pidX_.initialize(ros::NodeHandle(pnh, "linear/xy"));
  twist_controller_.pidY_.initialize(ros::NodeHandle(pnh, "linear/xy"));
  twist_controller_.pidZ_.initialize(ros::NodeHandle(pnh, "linear/z"));
  twist_controller_.pidR_.initialize(ros::NodeHandle(pnh, "angular/xy"));
  twist_controller_.pidP_.initialize(ros::NodeHandle(pnh, "angular/xy"));
  twist_controller_.pidYaw_.initialize(ros::NodeHandle(pnh, "angular/z"));

  // Read parameters from rosparam.
  GetRosParameter(pnh, "velocity_gain/x",
                  twist_controller_.controller_parameters_.velocity_gain_.x(),
                  &twist_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  twist_controller_.controller_parameters_.velocity_gain_.y(),
                  &twist_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  twist_controller_.controller_parameters_.velocity_gain_.z(),
                  &twist_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(pnh, "attitude_gain/x",
                  twist_controller_.controller_parameters_.attitude_gain_.x(),
                  &twist_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  twist_controller_.controller_parameters_.attitude_gain_.y(),
                  &twist_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  twist_controller_.controller_parameters_.attitude_gain_.z(),
                  &twist_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  twist_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &twist_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  twist_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &twist_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  twist_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &twist_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &twist_controller_.vehicle_parameters_);

  GetVehicleParameters(pnh, &twist_controller_.vehicle_parameters_);
  twist_controller_.InitializeParameters();
}

void TwistControllerNode::MotorStatusCallback(const std_msgs::Bool& motor_status_msg) {
  twist_controller_.ControllerActivation(motor_status_msg.data);
}

void TwistControllerNode::ClockCallback(const rosgraph_msgs::Clock& clock) {
  double t_in_sec_now;
  t_in_sec_now = clock.clock.toSec();
  if (dt_in_sec == 0.0)
  {dt_in_sec_auto = 0.001;} else {dt_in_sec_auto = t_in_sec_now - t_in_sec;}
  t_in_sec = t_in_sec_now;
  twist_controller_.SetTime(t_in_sec);
}

void TwistControllerNode::StepsizeCallback(const std_msgs::Float64MultiArray& step_msg) {
  if (auto_step_size)
  {dt_in_sec = dt_in_sec_auto;}
  else
  {dt_in_sec = step_msg.data[1];}
  twist_controller_.SetPeriod(dt_in_sec);
}

// void TwistControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {}

void TwistControllerNode::CommandTwistCallback(const geometry_msgs::Twist& twist) {
    twist_controller_.SetCmdTwist(twist);
} // command twist



void TwistControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("TwistController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  twist_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  twist_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;
  actuator_msg->header.seq++;

  motor_velocity_reference_pub_.publish(actuator_msg);
} // self twist

void TwistControllerNode::ImuCallback(const sensor_msgs::Imu& imu) {
  twist_controller_.SetAccel(imu);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_controller_node");

  rotors_control::TwistControllerNode twist_controller_node;

  ros::spin();

  return 0;
}

