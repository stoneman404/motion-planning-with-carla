#include "vehicle_state/kinodynamic_state.hpp"
#include <ros/ros.h>
namespace vehicle_state {

KinoDynamicState::KinoDynamicState(double _x,
                                   double _y,
                                   double _z,
                                   double _theta,
                                   double _kappa,
                                   double _v,
                                   double _a,
                                   double _centripental_acc)
    : x(_x), y(_y), z(_z),
      theta(_theta), kappa(_kappa), v(_v), a(_a), centripental_acc(_centripental_acc) {}

KinoDynamicState KinoDynamicState::GetNextStateAfterTime(double predict_time) const {
  double dt = 0.1;
  ROS_ASSERT(predict_time > 0.0);

  double cur_x = x;
  double cur_y = y;
  double cur_theta = theta;
  double cur_kappa = kappa;
  double cur_v = v;
  double cur_a = a;
  double next_x = this->x;
  double next_y = this->y;
  double next_z = this->z;
  double next_v = this->v;

  double next_a = this->a;
  double next_theta = this->theta;
  double next_kappa = this->kappa;
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
  return (KinoDynamicState(next_x, next_y, next_z, next_theta, next_kappa, next_v, next_a, centripental_acc));
}
}

