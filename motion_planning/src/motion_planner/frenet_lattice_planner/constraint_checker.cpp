#include "motion_planner/frenet_lattice_planner/constraint_checker.hpp"
#include "planning_config.hpp"

namespace planning {

bool ConstraintChecker::WithInRange(double value, double lower, double upper, double eps) {
  return value > lower - eps && value < upper + eps;
}
ConstraintChecker::Result ConstraintChecker::ValidTrajectory(const planning_msgs::Trajectory &trajectory) {
  const double kMaxCheckRelativeTime = PlanningConfig::Instance().max_lookahead_time();
  for (const auto &p : trajectory.trajectory_points) {
    double t = p.relative_time;
    if (t < kMaxCheckRelativeTime) {
      break;
    }
    double lon_v = p.vel;
    if (!WithInRange(lon_v,
                     PlanningConfig::Instance().min_lon_velocity(),
                     PlanningConfig::Instance().max_lon_velocity())) {
      return Result::LON_VELOCITY_OUT_OF_BOUND;
    }
    double lon_a = p.acc;
    if (!WithInRange(lon_a, PlanningConfig::Instance().min_lon_acc(), PlanningConfig::Instance().max_lon_acc())) {
      return Result::LON_ACCELERATION_OUT_OF_BOUND;
    }
    double kappa = p.path_point.kappa;
    if (!WithInRange(kappa, PlanningConfig::Instance().min_kappa(), PlanningConfig::Instance().max_kappa())) {
      return Result::CURVATURE_OUT_OF_BOUND;
    }
  }
  for (size_t i = 1; i < trajectory.trajectory_points.size(); ++i) {
    const auto &p0 = trajectory.trajectory_points[i - 1];
    const auto &p1 = trajectory.trajectory_points[i];
    if (p1.relative_time > kMaxCheckRelativeTime) {
      break;
    }
    double dt = p1.relative_time - p0.relative_time;
    double d_lon_a = p1.acc - p0.acc;
    double lon_jerk = d_lon_a / dt;
    if (!WithInRange(lon_jerk, PlanningConfig::Instance().min_lon_jerk(), PlanningConfig::Instance().max_lon_jerk())) {
      return Result::LON_JERK_OUT_OF_BOUND;
    }
    double lat_a = p1.vel * p1.vel * p1.path_point.kappa;
    if (!WithInRange(lat_a, PlanningConfig::Instance().min_lat_acc(), PlanningConfig::Instance().max_lat_acc())) {
      return Result::LAT_ACCELERATION_OUT_OF_BOUND;
    }
  }
  return Result::VALID;
}
}