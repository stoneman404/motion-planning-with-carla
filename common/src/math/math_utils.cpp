#include <Eigen/Geometry>
#include <planning_msgs/TrajectoryPoint.h>
#include "math/math_utils.hpp"

namespace common {

double MathUtils::NormalizeAngle(const double &angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double MathUtils::CalcAngleDist(double from, double to) { return NormalizeAngle(to - from); }

double MathUtils::slerp(double a0, double t0, double a1, double t1,
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

planning_msgs::TrajectoryPoint MathUtils::InterpolateTrajectoryPoint(
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

double MathUtils::lerp(const double &x0, const double &t0,
                       const double &x1, const double &t1,
                       const double &t) {
  if (std::fabs(t1 - t0) <= 1e-6) {
    return x0;
  }
  double ratio = (t - t0) / (t1 - t0);
  double x = x0 + ratio * (x1 - x0);
  return x;
}

double MathUtils::CalcKappa(double dx, double dy, double ddx, double ddy) {
  const double u = dx * ddy - dy * ddx;
  const double v = dx * dx + dy * dy;
  const double sqrt_v = std::sqrt(v);
  return u / (v * sqrt_v);
}

double MathUtils::CalcDKappa(double dx, double dy, double ddx, double ddy, double dddx, double dddy) {
  const double a = dx * ddy - dy * ddx;
  const double b = dx * dddy - dy * dddx;
  const double c = dx * ddx + dy * ddy;
  const double d = dx * dx + dy * dy;

  return (b * d - 3.0 * a * c) / (d * d * d);
}

Eigen::Vector3d MathUtils::Transform(const geometry_msgs::Pose &local_coordinate_transform,
                                     const Eigen::Vector3d &in_point) {
  Eigen::Matrix4d transitional_matrix;
  Eigen::Quaternion<double>
      quaternion(local_coordinate_transform.orientation.w, local_coordinate_transform.orientation.x,
                 local_coordinate_transform.orientation.y, local_coordinate_transform.orientation.z);
  Eigen::Matrix3d R = quaternion.toRotationMatrix();
  transitional_matrix << R(0, 0), R(0, 1), R(0, 2), local_coordinate_transform.position.x,
      R(1, 0), R(1, 1), R(1, 2), local_coordinate_transform.position.y,
      R(2, 0), R(2, 1), R(2, 2), local_coordinate_transform.position.z,
      0, 0, 0, 1;

  Eigen::Vector4d transitional_point;
  transitional_point << in_point.x(), in_point.y(), in_point.z(), 1;
  Eigen::Vector4d out_transitional_point = transitional_matrix.transpose() * transitional_point;
  return {out_transitional_point(0),
          out_transitional_point(1),
          out_transitional_point(2)};
}

double MathUtils::CrossProd(const Eigen::Vector2d &start_point,
                            const Eigen::Vector2d &end_point1,
                            const Eigen::Vector2d &end_point2) {
  Eigen::Vector2d vec_1 = end_point1 - start_point;
  Eigen::Vector2d vec_2 = end_point2 - start_point;
  return vec_1.x() * vec_2.y() - vec_1.y() * vec_2.x();
}

}
