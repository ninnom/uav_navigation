
#include <rotors_control/generic_pid.h>

namespace rotors_control {

GenericPID::GenericPID(const parameters& parameters)
  : parameters_(parameters)
{}

GenericPID::GenericPID() {}
GenericPID::~GenericPID() {}

void GenericPID::initialize(const ros::NodeHandle &param_nh)
{
 /* GetRosParameter(param_nh, "activated", parameters_.activated, parameters_.activated);
  GetRosParameter(param_nh, "differential", parameters_.activated, parameters_.differential;
  GetRosParameter(param_nh, "antiwindup", parameters_.activated, parameters_.antiwindup;
  GetRosParameter(param_nh, "k_p", parameters_.activated, parameters_.k_p);
  GetRosParameter(param_nh, "k_i", parameters_.activated, parameters_.k_i);
  GetRosParameter(param_nh, "k_d", parameters_.activated, parameters_.k_d);
  GetRosParameter(param_nh, "k_dd", parameters_.activated, parameters_.k_dd);
  GetRosParameter(param_nh, "limit_i", parameters_.activated, parameters_.limit_i);
  GetRosParameter(param_nh, "limit_out", parameters_.activated, parameters_.limit_out);
  GetRosParameter(param_nh, "time_const_ref", parameters_.activated, parameters_.time_const_ref);
  GetRosParameter(param_nh, "time_const_meas", parameters_.activated, parameters_.time_const_meas);
  GetRosParameter(param_nh, "period", parameters_.activated, parameters_.period); */

  param_nh.getParam("activated", parameters_.activated);
  param_nh.getParam("differential", parameters_.differential);
  param_nh.getParam("antiwindup", parameters_.antiwindup);
  param_nh.getParam("k_p", parameters_.k_p);
  param_nh.getParam("k_i", parameters_.k_i);
  param_nh.getParam("k_d", parameters_.k_d);
  param_nh.getParam("k_dd", parameters_.k_dd);
  param_nh.getParam("limit_i", parameters_.limit_i);
  param_nh.getParam("limit_out", parameters_.limit_out);
  param_nh.getParam("time_const_ref", parameters_.time_const_ref);
  param_nh.getParam("time_const_meas", parameters_.time_const_meas);
  param_nh.getParam("period", parameters_.period);
}

void GenericPID::reset()
{
  state_ = state();
}

template <typename T> static inline T& checknan(T& value)
{
  if (std::isnan(value)) value = T();
  return value;
}

// differential tracker
void GenericPID::differential_tracker(double x, double dx, double x_f, double dx_f, double *filtered, const double& tau_c,  const double& dt) {
  if (tau_c == 0.0)
  {
    x_f = x;
    dx_f = dx;
  }
  else
  {
    dx_f =  (x - x_f)/tau_c;
    x_f += dx_f*dt;
  }
  filtered[0] = x_f;
  filtered[1] = dx_f;
}

void GenericPID::track(double x_ref, double dx_ref, double *filtered, const double& tau_c, const double& dt)
{
  double x_ref_f[2];
  differential_tracker(x_ref, dx_ref, state_.x_ref, state_.dx_ref, x_ref_f, tau_c, dt);
  state_.x_ref = x_ref_f[0];
  state_.dx_ref = x_ref_f[1];
  filtered[0] = state_.x_ref;
  filtered[1] = state_.dx_ref;
}

double GenericPID::update_f(double e, double de, const double& dt)
{
  if (!parameters_.activated) return 0.0;
  double e_f[2];
  if (std::isnan(state_.e)) {state_.e = 0.0;}
  differential_tracker(e, 0.0, state_.e, state_.de, e_f, parameters_.time_const_ref, dt);
  state_.e = e_f[0];

  // std::cout<<"de is nan? "<<std::isnan(de)<<std::endl;
  if (std::isnan(de))
  {
    state_.de = e_f[1];
  }
  else
  {
    state_.de = de;
  }

  return update(state_.e, state_.de, 0.0, dt);
}

double GenericPID::update_f(double x_ref, double x, double dx, const double& dt)
{
  if (!parameters_.activated) return 0.0;

  double x_ref_f[2];
  differential_tracker(x_ref, 0.0, state_.x_ref, state_.dx_ref, x_ref_f, parameters_.time_const_ref, dt);
  state_.x_ref = x_ref_f[0];
  state_.dx_ref = x_ref_f[1];

  double x_f[2];
  differential_tracker(x, dx, state_.x, state_.dx, x_f, parameters_.time_const_meas, dt);
  state_.x = x_f[0];
  state_.dx = x_f[1];

  return update(state_.x_ref - state_.x, state_.dx_ref - state_.dx, 0.0, dt);
}

double GenericPID::update(double e, double de, double dde, const double& dt)
{
   parameters_.period = dt;

  if (!parameters_.activated) return 0.0;
  if (std::isnan(e)) return 0.0;

  // integral error
  state_.i += e * dt;
     // maxum of integral part
     if (parameters_.limit_i > 0.0)
     {
       if (state_.i >  parameters_.limit_i) state_.i =  parameters_.limit_i;
       if (state_.i < -parameters_.limit_i) state_.i = -parameters_.limit_i;
     }

  // differential error
  state_.d = de;

  // differential error
  state_.dd = dde;

  // proportional error
  state_.p = e;

  // calculate output...
  double output = parameters_.k_p * state_.p
                                + parameters_.k_i * state_.i
                                + parameters_.k_d * state_.d
                                + parameters_.k_dd * state_.dd;

  // antiwindup
  int antiwindup = 0;
  if (parameters_.limit_out > 0.0)
  {
    if (output >  parameters_.limit_out) { output =  parameters_.limit_out; antiwindup =  1; }
    if (output < -parameters_.limit_out) { output = -parameters_.limit_out; antiwindup = -1; }
  }
  if (parameters_.antiwindup && (e * dt * antiwindup > 0.0))
  {
    state_.i -= e * dt; output -= e*dt;
  }

  checknan(output);
  return output;
}

}

/*
double x_f[2];
pid_test_.differential_tracker(10.0, 0.0, streck_f, dstreck_f, x_f, 0.01, dt_in_sec);
streck_f = x_f[0];
dstreck_f = x_f[1];

std::cout<<"time: "<<t_in_sec<<" reference: "<<10.0<<", streck: "<<streck_f<<", dstreck_f: "<<dstreck_f<<std::endl;
*/
