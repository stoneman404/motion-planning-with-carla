#include "math_utils.hpp"

namespace planning {

double NormalizeAngle(const double &angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double CalcAngleDist(double from, double to) { return NormalizeAngle(to - from); }

double slerp(double a0, double t0, double a1, double t1,
             double t) {
  if (std::abs(t1 - t0) <= 1e-6) {
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

planning_msgs::TrajectoryPoint InterpolateTrajectoryPoint(
    const planning_msgs::TrajectoryPoint &p0,
    const planning_msgs::TrajectoryPoint &p1, double time) {
  planning_msgs::TrajectoryPoint tp;
  tp.relative_time = time;
  const double t0 = p0.relative_time;
  const double t1 = p1.relative_time;
  tp.vel = lerp(p0.vel, t0, p1.vel, t1, time);
  tp.acc = lerp(p0.acc, t0, p1.acc, t1, time);
  tp.jerk = lerp(p0.jerk, t0, p1.jerk, t1, time);
  tp.steer_angle = lerp(p0.steer_angle, t0, p1.steer_angle, t1, time);
  tp.path_point.x = lerp(p0.path_point.x, t0, p1.path_point.x, t1, time);
  tp.path_point.y = lerp(p0.path_point.y, t0, p1.path_point.y, t1, time);
  tp.path_point.s = lerp(p0.path_point.s, t0, p1.path_point.s, t1, time);
  tp.path_point.theta = slerp(p0.path_point.theta, t0, p1.path_point.theta, t1, time);
  tp.path_point.kappa = lerp(p0.path_point.kappa, t0, p1.path_point.kappa, t1, time);
  tp.path_point.dkappa = lerp(p0.path_point.dkappa, t0, p1.path_point.dkappa, t1, time);
  return tp;

}
double lerp(const double &x0, const double &t0,
            const double &x1, const double &t1,
            const double &t) {
  if (std::fabs(t1 - t0) <= 1e-6) {
    return x0;
  }
  double ratio = (t - t0) / (t1 - t0);
  double x = x0 + ratio * (x1 - x0);
  return x;
}

double CalcKappa(double dx, double dy, double ddx, double ddy) {
  const double u = dx * ddy - dy * ddx;
  const double v = dx * dx + dy * dy;
  const double sqrt_v = std::sqrt(v);
  return u / (v * sqrt_v);
}

double CalcDKappa(double dx, double dy, double ddx, double ddy, double dddx, double dddy) {
  const double a = dx * dddy - dy * dddx;
  const double b = dx * dx + dy * dy;
  const double c = dx * ddy - dy * ddx;
  const double d = dx * ddx + dy * ddy;
  const double sqrt_b = std::sqrt(b);
  return (a * b - 3 * c * d) / (b * b * sqrt_b);
}
}
