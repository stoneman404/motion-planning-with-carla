#include "vehicle_state/kinodynamic_state.hpp"
#include <ros/ros.h>
namespace vehicle_state {

KinoDynamicState::KinoDynamicState(double x,
                                   double y,
                                   double z,
                                   double theta,
                                   double kappa,
                                   double v,
                                   double a,
                                   double centripental_acc)
    : x_(x), y_(y), z_(z),
      theta_(theta), kappa_(kappa), v_(v), a_(a), centripental_acc_(centripental_acc) {}
KinoDynamicState KinoDynamicState::GetNextStateAfterTime(double predict_time) const {
  double dt = 0.05;
  ROS_ASSERT(predict_time > 0.0);

  double cur_x = x_;
  double cur_y = y_;
  double cur_theta = theta_;
  double cur_kappa = kappa_;
  double cur_v = v_;
  double cur_a = a_;
  double next_x = this->x_;
  double next_y = this->y_;
  double next_z = this->z_;
  double next_v = this->v_;

  double next_a = this->a_;
  double next_theta = this->theta_;
  double next_kappa = this->kappa_;
  if (dt > predict_time) {
    dt = predict_time;
  }
  double t = dt;
  while (t <= predict_time + 1e-8) {
    t += dt;
    double intermidiate_theta = cur_theta + 0.5 * dt * cur_v * cur_kappa;
    next_theta = cur_theta + dt * (cur_v + 0.5 * cur_a * dt) * cur_kappa;
    next_x = cur_x + dt * (cur_v + 0.5 * cur_a * dt) * std::cos(intermidiate_theta);
    next_y = cur_y + dt * (cur_v + 0.5 * cur_a * dt) * std::sin(intermidiate_theta);
    next_v = (cur_v + 0.5 * cur_a * dt);
    cur_x = next_x;
    cur_y = next_y;
    cur_theta = next_theta;
    cur_v = next_v;
  }
  return (KinoDynamicState(next_x, next_y, next_z, next_theta, next_kappa, next_v, next_a, centripental_acc_));
}
}

