#include "pure_pursuit_control.hpp"
#include <math/math_utils.hpp>
#include <boost/numeric/odeint.hpp>
namespace planning {
bool PurePursuitControl::CalculateDesiredSteer(const double wheel_base_len,
                                               const double angle_diff,
                                               const double lookahead_dist,
                                               double *steer) {
  *steer = std::atan2(2.0 * wheel_base_len * std::sin(angle_diff), lookahead_dist);
  return true;
}

IdealSteerModel::IdealSteerModel(double wheelbase_len,
                                 double max_lon_acc,
                                 double max_lon_dec,
                                 double max_lon_acc_jerk,
                                 double max_lon_dec_jerk,
                                 double max_lat_acc,
                                 double max_lat_jerk,
                                 double max_steering_angle,
                                 double max_steer_rate,
                                 double max_curvature)
    : wheelbase_len_(wheelbase_len),
      max_lon_acc_(max_lon_acc),
      max_lon_decel_(max_lon_dec),
      max_lon_acc_jerk_(max_lon_acc_jerk),
      max_lon_decel_jerk_(max_lon_dec_jerk),
      max_lat_acc_(max_lat_acc),
      max_lat_jerk_(max_lat_jerk),
      max_steering_(max_steering_angle),
      max_steer_rate_(max_steer_rate),
      max_curvature_(max_curvature) {}

vehicle_state::KinoDynamicState IdealSteerModel::Step(const Control &control, const vehicle_state::KinoDynamicState &state, double dt) {
  state_ = state;
  control_ = control;
  state_.steer = std::atan(state.kappa * wheelbase_len_);
  control_.velocity = std::max(0.0, control_.velocity);
  control_.steer = std::min(std::max(control_.steer, -max_steering_), max_steering_);
  ClampControl(dt);
  desired_lon_acc_ = (control_.velocity - state_.v) / dt;
  desired_steer_rate_ = common::MathUtils::NormalizeAngle(control_.steer - state_.steer) / dt;
  std::array<double, 5> inter_state{state.x, state.y, state.theta, state.v, state.steer};
  boost::numeric::odeint::integrate(boost::ref(*this), inter_state, 0.0, dt, dt);
  vehicle_state::KinoDynamicState updated_state{};
  updated_state.x = inter_state[0];
  updated_state.y = inter_state[1];
  updated_state.theta = inter_state[2];
  updated_state.v = inter_state[3];
  updated_state.steer = inter_state[4];
  return updated_state;
}

void IdealSteerModel::operator()(const std::array<double, 5> &x, std::array<double, 5> &dxdt, const double) {
  vehicle_state::KinoDynamicState state{};
  state.x = x[0];
  state.y = x[1];
  state.theta = x[2];
  state.v = x[3];
  state.steer = x[4];
  dxdt[0] = std::cos(state.theta) * state.v;
  dxdt[1] = std::sin(state.theta) * state.v;
  dxdt[2] = std::tan(state.steer) * state.v / wheelbase_len_;
  dxdt[3] = desired_lon_acc_;
  dxdt[4] = desired_steer_rate_;
}

void IdealSteerModel::ClampControl(double dt) {
  desired_lon_acc_ = (control_.velocity - state_.v) / dt;
  double desired_lon_jerk = (desired_lon_acc_ - state_.a) / dt;
  desired_lon_jerk =
      std::min(std::max(desired_lon_jerk, -max_lon_decel_jerk_), max_lon_acc_jerk_);
  desired_lon_acc_ = desired_lon_jerk * dt + state_.a;
  desired_lon_acc_ = std::min(std::max(desired_lon_acc_, -max_lon_decel_), max_lon_acc_);
  control_.velocity = std::max(state_.v + desired_lon_acc_ * dt, 0.0);

  desired_lat_acc_ =
      pow(control_.velocity, 2) * (tan(control_.steer) / wheelbase_len_);
  double lat_acc_ori = pow(state_.v, 2) * state_.kappa;
  double lat_jerk_desired = (desired_lat_acc_ - lat_acc_ori) / dt;
  lat_jerk_desired = std::max(std::min(lat_jerk_desired, -max_lat_jerk_), max_lat_jerk_);
  desired_lat_acc_ = lat_jerk_desired * dt + lat_acc_ori;
  desired_lat_acc_ = std::min(std::max(desired_lat_acc_, -max_lat_acc_), max_lat_acc_);
  control_.steer = atan(desired_lat_acc_ * wheelbase_len_ /
      std::max(pow(control_.velocity, 2), 0.1 * 1e-1));
  desired_steer_rate_ = common::MathUtils::NormalizeAngle(control_.steer - state_.steer) / dt;
  desired_steer_rate_ =
      std::min(std::max(desired_steer_rate_, -max_steer_rate_), max_steer_rate_);
  control_.steer = common::MathUtils::NormalizeAngle(state_.steer + desired_steer_rate_ * dt);
}
}
