
#ifndef TWIST_CONTROLLER_H
#define TWIST_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sensor_msgs/Imu.h>

#include "rotors_control/generic_pid.h"

#include <rotors_control/common.h>
//include "tud_firefly_control/tud_firefly_common.h"
#include <rotors_control/parameters.h>

#include <fstream>
#include <iostream>
#include <ostream>

namespace rotors_control{

// Default values for the roll pitch yawrate thrust controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultVelocityGainTwistControl = Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultAttitudeGainTwistControl  = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGainTwistControl  = Eigen::Vector3d(0.52, 0.52, 0.025);

class  TwistControllerParameters{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TwistControllerParameters()
   : velocity_gain_(kDefaultVelocityGainTwistControl ),
     attitude_gain_(kDefaultAttitudeGainTwistControl ),
     angular_rate_gain_(kDefaultAngularRateGainTwistControl)
  {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

Eigen::Matrix4Xd allocation_matrix_;
Eigen::Vector3d velocity_gain_;
Eigen::Vector3d attitude_gain_;
Eigen::Vector3d angular_rate_gain_;
RotorConfiguration rotor_configuration_;
};

class TwistController {
 public:
  TwistController();
  ~TwistController();

  void InitializeParameters();
  void reset();

  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities);
  void ControllerActivation(const bool& active);
  void SetOdometry(const EigenOdometry& odometry_msg);
  // void SetOdometryStamped(const EigenOdometry& odometry_msg, const ros::Time& t, const ros::Duration& dt);
  void SetTime(const double& t);
  void SetPeriod(const double& dt);
  void SetCmdTwist(const geometry_msgs::Twist& twist);
  void SetAccel(const sensor_msgs::Imu& imu);

  TwistControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;
  GenericPID pidX_;
  GenericPID pidY_;
  GenericPID pidZ_;
  GenericPID pidR_;
  GenericPID pidP_;
  GenericPID pidYaw_;

 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
   double dt_in_sec;
   double t_in_sec;

   Eigen::Vector3d gt_lin_accel_B;
   // Eigen::Vector3d cmd_twist_lin_W;
   Eigen::Vector3d cmd_twist_lin_B;
   double cmd_yaw_rate;

   Eigen::Matrix3d R;
   double roll;
   double pitch;
   double yaw;
   double load_factor;
   double rotor_velocities_max_;
   Eigen::VectorXd rotor_velocities_sqr_;

   bool bool_test_;

   bool initialized_params_;
   bool controller_active_;

   Eigen::Vector3d normalized_attitude_gain_;
   Eigen::Vector3d normalized_angular_rate_gain_;
   Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
   Eigen::Matrix4Xd rotor_velocities_to_UAV_accelerations_;

    Eigen::Matrix4Xd rotor_to_UAV_accel; // for test
   Eigen::VectorXd rotor_velocities_back_; // for test
   EigenOdometry odometry_;

   void ComputeDesiredAngularAcc(const Eigen::Vector3d & acceleration,
                               Eigen::Vector3d* angular_acceleration);
   void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration);

   std::ofstream fout;
};

}

#endif // _TWIST_CONTROLLER_H
