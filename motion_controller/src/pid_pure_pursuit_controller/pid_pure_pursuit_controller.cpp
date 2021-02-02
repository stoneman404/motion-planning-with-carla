#include <numeric>
#include <math/math_utils.hpp>
#include "pid_pure_pursuit_controller.hpp"

namespace control {

PIDPurePursuitController::PIDPurePursuitController(const ControlConfigs &control_configs, double loop_rate)
    : control_configs_(control_configs), loop_rate_(loop_rate) {
  lon_error_buffer_.set_capacity(15);
}

bool PIDPurePursuitController::Execute(double current_time_stamp,
                                       const vehicle_state::VehicleState &vehicle_state,
                                       const planning_msgs::Trajectory &trajectory,
                                       carla_msgs::CarlaEgoVehicleControl &control) {
  planning_msgs::TrajectoryPoint cur_tp, target_tp;
  auto kinodynamic_state = vehicle_state.GetKinoDynamicVehicleState();
  if (!GetMatchedPointByPosition(kinodynamic_state.x, kinodynamic_state.y, trajectory, cur_tp)) {
    return false;
  }
  if (cur_tp == trajectory.trajectory_points.back()) {
    return false;
  }
//  double preview_time = current_time_stamp + control_configs_.lookahead_time;
//  if (!GetMatchedPointByAbsoluteTime(preview_time, trajectory, target_tp)) {
//    return false;
//  }
//  double lookahead_distance = std::max(kinodynamic_state.v * control_configs_.lookahead_time, 15.0);
  double approx_lookahead_dist =
      std::min(std::max(control_configs_.pure_pursuit_configs.steer_control_min_lookahead_dist,
                        kinodynamic_state.v * control_configs_.pure_pursuit_configs.steer_control_gain),
               control_configs_.pure_pursuit_configs.steer_control_max_lookahead_dist);
  if (!GetMatchedPointByS(cur_tp.path_point.s + approx_lookahead_dist, trajectory, target_tp)) {
    return false;
  }
//  double max_vel = trajectory.trajectory_points.front().vel;
//  for (const auto& tp: trajectory.trajectory_points) {
//    if (tp.vel > max_vel) {
//      max_vel = tp.vel;
//    }
//  }

  const double cur_lon_s_dot = kinodynamic_state.v;
  const double target_lon_s_dot = std::min(target_tp.vel, (3.0 / std::fabs(target_tp.path_point.kappa) + 1e-4));

  double throttle = 0.0;
  double steer = 0.0;

  if (!LongitudinalControl(cur_lon_s_dot, target_lon_s_dot, 1.0 / loop_rate_, &throttle)) {
    return false;
  }
  double wheel_base = vehicle_state.vehicle_params().lf_ + vehicle_state.vehicle_params().lr_;
  double max_steer = vehicle_state.vehicle_params().max_steer_angle_;
  if (!PurePursuitSteerControl(max_steer, wheel_base, kinodynamic_state, target_tp, trajectory, &steer)) {
    return false;
  }

  control.steer = steer;
  control.throttle = throttle;
  control.brake = 0.0;
  control.hand_brake = false;
  control.manual_gear_shift = false;
  return true;
}

bool PIDPurePursuitController::LongitudinalControl(double current_speed, double target_speed,
                                                   double delta_t, double *throttle) {
  double speed_error = (target_speed - current_speed) / 3.6;
  this->lon_error_buffer_.push_back(speed_error);
  double speed_error_rate = 0.0;
  double speed_error_intergal = 0.0;
  if (lon_error_buffer_.size() >= 2) {
    speed_error_rate = (lon_error_buffer_.back() - *(lon_error_buffer_.end() - 2)) / delta_t;
    speed_error_intergal = std::accumulate(lon_error_buffer_.begin(), lon_error_buffer_.end(), 0.0) * delta_t;
  }
  *throttle = Clamp<double>(
      control_configs_.lon_configs.lon_kp * speed_error
          + control_configs_.lon_configs.lon_kd * speed_error_rate / delta_t
          + control_configs_.lon_configs.lon_ki * speed_error_intergal * delta_t, 0.0, 1.0);
  return true;
}
//
//bool PIDPurePursuitController::LateralControl(const Eigen::Vector3d &current_pose,
//                                              const Eigen::Vector3d &target_pose,
//                                              double delta_t,
//                                              double *steer) {
//  const double cur_x = current_pose.x();
//  const double cur_y = current_pose.y();
//  const double cur_theta = current_pose.z();
//  const double target_x = target_pose.x();
//  const double target_y = target_pose.y();
//  Eigen::Vector2d cur_heading{std::cos(cur_theta), std::sin(cur_theta)};
//  Eigen::Vector2d to_target{target_x - cur_x, target_y - cur_y};
//  double cross_prod = cur_heading.x() * to_target.y() - cur_heading.y() * to_target.x();
//  double dot_prod = cur_heading.x() * to_target.x() + cur_heading.y() * to_target.y();
//  double angle_error = std::acos(
//      Clamp<double>(dot_prod / (to_target.norm() * cur_heading.norm()), -1.0, 1.0));
//  angle_error = cross_prod < 0.0 ? -angle_error : angle_error;
//  this->lat_error_buffer_.push_back(angle_error);
//  double differential_angle_error = 0.0;
//  double intergal_angle_error = 0.0;
//  if (lat_error_buffer_.size() >= 2) {
//    differential_angle_error = (lat_error_buffer_.back() - *(lat_error_buffer_.end() - 2)) / delta_t;
//    intergal_angle_error = std::accumulate(lat_error_buffer_.begin(), lat_error_buffer_.end(), 0.0) * delta_t;
//  } else {
//    differential_angle_error = 0.0;
//    intergal_angle_error = 0.0;
//  }
//  *steer = Clamp<double>(control_configs_.lat_configs.lat_kp * angle_error
//                             + control_configs_.lat_configs.lat_kd * differential_angle_error / delta_t
//                             + control_configs_.lat_configs.lat_ki * intergal_angle_error * delta_t, -1.0, 1.0);
//  return true;
//}

template<class T>
T PIDPurePursuitController::Clamp(const T &value, const T &lower, const T &upper) {
  ROS_ASSERT(upper > lower);
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}
bool PIDPurePursuitController::GetMatchedPointByPosition(double x,
                                                         double y,
                                                         const planning_msgs::Trajectory &trajectory,
                                                         planning_msgs::TrajectoryPoint &matched_tp) {
  if (trajectory.trajectory_points.empty()) {
    return false;
  }
  size_t min_index = 0;
  double min_sqr_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < trajectory.trajectory_points.size(); ++i) {
    auto pp = trajectory.trajectory_points.at(i).path_point;
    double sqr_dist = (pp.x - x) * (pp.x - x) + (pp.y - y) * (pp.y - y);
    if (sqr_dist < min_sqr_dist) {
      min_sqr_dist = sqr_dist;
      min_index = i;
    }
  }
  matched_tp = trajectory.trajectory_points.at(min_index);
  return true;
}

bool PIDPurePursuitController::GetMatchedPointByAbsoluteTime(double &time_stamp,
                                                             const planning_msgs::Trajectory &trajectory,
                                                             planning_msgs::TrajectoryPoint &matched_tp) {
  auto trajectory_time_stamp = trajectory.header.stamp;
  double relative_time = time_stamp - trajectory_time_stamp.toSec();
  return PIDPurePursuitController::GetMatchedPointByRelativeTime(relative_time, trajectory, matched_tp);
}

bool PIDPurePursuitController::GetMatchedPointByRelativeTime(double relative_time,
                                                             const planning_msgs::Trajectory &trajectory,
                                                             planning_msgs::TrajectoryPoint &matched_tp) {
  if (trajectory.trajectory_points.empty()) {
    return false;
  }
  auto it_low =
      std::lower_bound(trajectory.trajectory_points.begin(), trajectory.trajectory_points.end(), relative_time,
                       [](const planning_msgs::TrajectoryPoint &tp, double relative_time) -> bool {
                         return tp.relative_time < relative_time;
                       });
  if (it_low == trajectory.trajectory_points.begin()) {
    matched_tp = *it_low;
    return true;
  }
  if (it_low == trajectory.trajectory_points.end()) {
    matched_tp = trajectory.trajectory_points.back();
    return true;
  }
  auto it_lower = it_low - 1;
  if (relative_time - it_lower->relative_time < it_low->relative_time - relative_time) {
    matched_tp = *it_lower;
  } else {
    matched_tp = *it_low;
  }
  return true;
}
bool PIDPurePursuitController::GetMatchedPointByS(double s,
                                                  const planning_msgs::Trajectory &trajectory,
                                                  planning_msgs::TrajectoryPoint &matched_tp) {
  if (trajectory.trajectory_points.empty()) {
    return false;
  }
  auto it_low =
      std::lower_bound(trajectory.trajectory_points.begin(), trajectory.trajectory_points.end(),
                       s, [](const planning_msgs::TrajectoryPoint &tp, double s) -> bool {
            return tp.path_point.s < s;
          });
  if (it_low == trajectory.trajectory_points.begin()) {
    matched_tp = *it_low;
    return true;
  }
  if (it_low == trajectory.trajectory_points.end()) {
    matched_tp = trajectory.trajectory_points.back();
    return true;
  }
  auto it_lower = it_low - 1;
  if (s - it_lower->path_point.s < it_low->path_point.s - s) {
    matched_tp = *it_lower;
  } else {
    matched_tp = *it_low;
  }
  return true;
}

bool PIDPurePursuitController::PurePursuitSteerControl(double max_steer,
                                                       double wheelbase,
                                                       const vehicle_state::KinoDynamicState &vehicle_state,
                                                       const planning_msgs::TrajectoryPoint &target_tp,
                                                       const planning_msgs::Trajectory &trajectory,
                                                       double *steer) {
  Eigen::Vector2d cur_xy{vehicle_state.x, vehicle_state.y};
  Eigen::Vector2d destination{target_tp.path_point.x, target_tp.path_point.y};
  double lookahead_dist = (destination - cur_xy).norm();
  double cur_to_dest_angle = common::MathUtils::NormalizeAngle(
      std::atan2(destination.y() - cur_xy.y(),
                 destination.x() - cur_xy.x()));
  double angle_diff = common::MathUtils::CalcAngleDist(vehicle_state.theta, cur_to_dest_angle);
  double steer_tmp = std::atan2(2.0 * wheelbase * std::sin(angle_diff), lookahead_dist);
  double normalized_steer = steer_tmp / max_steer;
  *steer = Clamp<double>(normalized_steer, -1.0, 1.0);
  return true;
}

}