#ifndef GENERIC_PID_H
#define GENERIC_PID_H

#include <ros/ros.h>
#include <limits>
// #include <rotors_control/parameters_ros.h>

namespace rotors_control{

class GenericPID // Generic PID controller
{
public:
  struct parameters {
    parameters()
      : activated(true)
      , differential(true)
      , antiwindup(true)
      , time_const_ref(0.01)
      , time_const_meas(0.01)
      , k_p(0.0)
      , k_i(0.0)
      , k_d(0.0)
      , k_dd(0.0)
      , limit_i(100.0)
      , limit_out(100.0)
     //, limit_i(std::numeric_limits<double>::quiet_NaN())
     //, limit_out(std::numeric_limits<double>::quiet_NaN())
      , period(-1.0) {}

    bool activated;
    bool differential;
    bool antiwindup;
    double time_const_ref;
    double time_const_meas;
    double k_p, k_i, k_d, k_dd;
    double limit_i, limit_out;
    double period;
  } parameters_;

  struct state {
    state()
      : e(std::numeric_limits<double>::quiet_NaN())
      , de(0.0)
      , dde(0.0)
      , ff(0.0)
      , p(std::numeric_limits<double>::quiet_NaN())
      , i(0.0)
      , d(std::numeric_limits<double>::quiet_NaN())
      , dd(std::numeric_limits<double>::quiet_NaN())
      , time_sim_old(std::numeric_limits<double>::quiet_NaN()) {}

    double x_ref, dx_ref, x, dx, e, de, dde, ff; // setpoint and velocity, measurement and velocity, error in p, i, d and feed forward
    double p, i, d, dd; // outputs
    double time_sim_old;
  } state_;

public:
  GenericPID();
  GenericPID(const parameters& parameters);
  ~GenericPID();

  void initialize(const ros::NodeHandle &param_nh);
  void reset();

  void differential_tracker(double x, double dx, double x_f, double dx_f, double *filtered, const double& tau_c, const double& dt);

  void track(double x, double dx, double *filtered, const double& tau_c, const double& dt);

  double update_f(double e, double de, const double& dt);

  double update_f(double x_ref, double x, double dx, const double& dt);

  double update(double e, double de, double dde, const double& dt);
};

}

#endif // GENERIC_PID_H
