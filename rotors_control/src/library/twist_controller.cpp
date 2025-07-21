#include "rotors_control/twist_controller.h"

namespace rotors_control {

TwistController::TwistController()
    : initialized_params_(false),
      controller_active_(false),
      rotor_velocities_max_(838.0)
{
  bool_test_ = false;
 InitializeParameters();
 rotor_velocities_sqr_.resize(6);
}

TwistController::~TwistController() {}

void TwistController::InitializeParameters() {

  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));

 normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
     * vehicle_parameters_.inertia_.inverse();
 // To make the tuning independent of the inertia matrix we divide here.
 normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
     * vehicle_parameters_.inertia_.inverse();

 // Read inertial.
 Eigen::Matrix4d I;
 I.setZero();
 I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
 I(3, 3) = 1;
//  angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  angular_acc_to_rotor_velocities_.resize(6, 4);
 // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
 // A^{ \dagger} = A^T*(A*A^T)^{-1}
 angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
     * (controller_parameters_.allocation_matrix_
     * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;

 Eigen::Matrix4d inertia_mass_matrix_;
 inertia_mass_matrix_.setZero();
 inertia_mass_matrix_.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
 inertia_mass_matrix_(3, 3) = vehicle_parameters_.mass_;
 rotor_velocities_to_UAV_accelerations_.resize(4, 6);
 rotor_velocities_to_UAV_accelerations_.setZero();
 for (int i=0;i<rotor_velocities_to_UAV_accelerations_.rows();i++)
 {
   rotor_velocities_to_UAV_accelerations_.block<1, 6>(i,0) = controller_parameters_.allocation_matrix_.block<1, 6>(i,0)/inertia_mass_matrix_(i,i);
 }

 rotor_to_UAV_accel = angular_acc_to_rotor_velocities_.transpose()
     *(angular_acc_to_rotor_velocities_ * angular_acc_to_rotor_velocities_.transpose()).inverse();

 // Initialization is over.
 initialized_params_ = true;
 // controller_active_ = true;
}

void TwistController::reset() {
 pidX_.reset();
 pidY_.reset();
 pidZ_.reset();
 pidR_.reset();
 pidP_.reset();
 pidY_.reset();
}

void TwistController::ControllerActivation(const bool& active)
{
    controller_active_ = active;
}

void TwistController::SetTime(const double &t)
{
    t_in_sec = t;
}

void TwistController::SetPeriod(const double &dt)
{
    dt_in_sec = dt;
}

void TwistController::SetCmdTwist(const geometry_msgs::Twist& twist)
{
  cmd_twist_lin_B << twist.linear.x, twist.linear.y, twist.linear.z;
  cmd_yaw_rate = twist.angular.z;
}

void TwistController::SetAccel(const sensor_msgs::Imu& imu)
{
  gt_lin_accel_B << imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z;
}

void TwistController::SetOdometry(const EigenOdometry& odometry) {
 odometry_ = odometry;
 R = odometry_.orientation.toRotationMatrix();

 double w = odometry_.orientation.w();
 double x =  odometry_.orientation.x();
 double y =  odometry_.orientation.y();
 double z = odometry_.orientation.z();

 load_factor = 1. / (w*w - x*x - y*y + z*z);
 roll  =  atan2(2.*y*z + 2.*w*x, z*z - y*y - x*x + w*w);
 pitch = -asin(2.*x*z - 2.*w*y);
 yaw =  atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
}

void TwistController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());

  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d acceleration;
  ComputeDesiredAcceleration(&acceleration);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

  // Project thrust onto body z axis.
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;
  // std::cout<<"Vor Ap:"<<angular_acceleration_thrust(0)<<", Aq:"<<angular_acceleration_thrust(1)<<", Ar:"<<angular_acceleration_thrust(2)<<", Az:"<<angular_acceleration_thrust(3)/vehicle_parameters_.mass_<<std::endl;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  rotor_velocities_sqr_ = *rotor_velocities;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
  *rotor_velocities = rotor_velocities->cwiseMin(rotor_velocities_max_);

  Eigen::MatrixXd test(6, 6);
  test = rotor_velocities->asDiagonal();
  Eigen::FullPivLU<Eigen::MatrixXd> test_(test);
  if (test_.rank() < 6)
  {
   // std::cout<<"rotor_velocities has 0 element: "<<std::endl
   //                 <<*rotor_velocities<<std::endl
   //                  <<"corresponding accels are:"<<std::endl
   //                  <<angular_acceleration_thrust<<std::endl;
  }
}

void TwistController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) {
  assert(acceleration);

  // Transform velocity to world frame (But only yaw).
  // const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
  // Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  // Eigen::Vector3d velocity_B = odometry_.velocity;
  Eigen::Vector3d velocity_W =  R * odometry_.velocity;
  Eigen::Vector3d cmd_twist_lin_W;
  cmd_twist_lin_W <<  cos(yaw)*cmd_twist_lin_B(0) - sin(yaw)*cmd_twist_lin_B(1),
                                            sin(yaw)*cmd_twist_lin_B(0) + cos(yaw)*cmd_twist_lin_B(1),
                                            cmd_twist_lin_B(2);

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

  *acceleration << -pidX_.update_f(cmd_twist_lin_W(0), velocity_W(0), 0.0, dt_in_sec),
                                    -pidY_.update_f(cmd_twist_lin_W(1), velocity_W(1), 0.0, dt_in_sec),
                                    -pidZ_.update_f(cmd_twist_lin_W(2), velocity_W(2), 0.0, dt_in_sec) - vehicle_parameters_.gravity_*e_3(2);
  }

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void TwistController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration) {

  assert(angular_acceleration);

  /* Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
  R = odometry_.orientation.toRotationMatrix(); */

  double w = odometry_.orientation.w();
  double x =  odometry_.orientation.x();
  double y =  odometry_.orientation.y();
  double z = odometry_.orientation.z();
  // std::cout<<"x:"<<x<<"y:"<<y<<"z:"<<z<<"w:"<<w<<std::endl;

  load_factor = 1. / (w*w - x*x - y*y + z*z);
  roll  =  atan2(2.*y*z + 2.*w*x, z*z - y*y - x*x + w*w);
  pitch = -asin(2.*x*z - 2.*w*y);
  yaw =  atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);

  // std::cout<<"roll, pitch, yaw"<<std::endl;
  // std::cout<<"x:"<<x<<", y:"<<y<<", z:"<<z<<", w:"<<w<<std::endl;
  // std::cout<<"roll:"<<roll<<", pitch:"<<pitch<<", yaw:"<<yaw<<std::endl;

  //Eigen::Vector3d acceleration_B;

  //acceleration_B << cos(yaw)*acceleration(0) + sin(yaw)*acceleration(1),
  //                                   - sin(yaw)*acceleration(0) + cos(yaw)*acceleration(1),
  //                                    acceleration(2)*load_factor;

  //  *angular_acceleration << pidR_.update_f(0.0, roll, 0.0,dt_in_sec ),
  //                                                     pidP_.update_f(0.0, pitch, 0.0, dt_in_sec),
  //                                                     0.0;


  // Get the desired rotation matrix.
  Eigen::Vector3d b1_des;
  // double yaw_des = command_trajectory_.getYaw();
  double yaw_des = yaw;
  b1_des << cos(yaw_des), sin(yaw_des), 0;

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  Eigen::Matrix3d R_des;
  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;

  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  // angular_rate_des[2] = command_trajectory_.getYawRate();
  angular_rate_des[2] = cmd_yaw_rate;

  //std::cout<<"cmd_yate_rate is: "<<std::endl;  std::cout<<cmd_yaw_rate<<std::endl;

  Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                           - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                           + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
 }

}
