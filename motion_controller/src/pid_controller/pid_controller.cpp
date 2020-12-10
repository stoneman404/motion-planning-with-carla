#include <numeric>
#include "pid_controller.hpp"

namespace control {

PIDController::PIDController(const PIDConfigs &pid_configs, double loop_rate)
    : pid_configs_(pid_configs), loop_rate_(loop_rate) {
  lat_error_buffer_.set_capacity(10);
  lon_error_buffer_.set_capacity(30);
}

bool PIDController::Execute(double current_time_stamp,
                            const vehicle_state::VehicleState &vehicle_state,
                            const planning_msgs::Trajectory &trajectory,
                            carla_msgs::CarlaEgoVehicleControl &control) {
  planning_msgs::TrajectoryPoint cur_tp, target_tp;
  auto kinodynamic_state = vehicle_state.GetKinoDynamicVehicleState();
  if (!GetMatchedPointByPosition(kinodynamic_state.x_, kinodynamic_state.y_, trajectory, cur_tp)) {
    return false;
  }
  double preview_time = current_time_stamp + pid_configs_.lookahead_time;
  if (!GetMatchedPointByAbsoluteTime(preview_time, trajectory, target_tp)) {
    return false;
  }
  const double delta_theta = kinodynamic_state.theta_ - cur_tp.path_point.theta;
  const double cos_delta_theta = std::cos(delta_theta);
  const double cur_lon_s_dot = kinodynamic_state.v_ * cos_delta_theta;
  const double target_lon_s_dot = target_tp.vel;
  double throttle = 0.0;
  double steer = 0.0;

  if (!LongitudinalControl(cur_lon_s_dot, target_lon_s_dot, 1. / loop_rate_, &throttle)) {
    return false;
  }

  if (!LateralControl({kinodynamic_state.x_, kinodynamic_state.y_, kinodynamic_state.theta_},
                      {target_tp.path_point.x, target_tp.path_point.y, target_tp
                          .path_point.theta},
                      1.0 / loop_rate_,
                      &steer)) {
    return false;
  }
  control.steer = steer;
  control.throttle = throttle;
  control.brake = 0.0;
  control.hand_brake = false;
  control.manual_gear_shift = false;
  return true;
}

bool PIDController::LongitudinalControl(double current_speed, double target_speed,
                                        double delta_t, double *throttle) {
  double speed_error = target_speed - current_speed;
  this->lon_error_buffer_.push_back(speed_error);
  double speed_error_rate = 0.0;
  double speed_error_intergal = 0.0;
  if (lon_error_buffer_.size() >= 2) {
    speed_error_rate = (lon_error_buffer_.back() - *(lon_error_buffer_.end() - 2)) / delta_t;
    speed_error_intergal = std::accumulate(lon_error_buffer_.begin(), lon_error_buffer_.end(), 0.0) * delta_t;
  }
  *throttle = Clamp<double>(
      pid_configs_.lon_configs.lon_kp * speed_error + pid_configs_.lon_configs.lon_kd * speed_error_rate / delta_t
          + pid_configs_.lon_configs.lon_ki * speed_error_intergal * delta_t, 0.0, 1.0);
  return true;
}

bool PIDController::LateralControl(const Eigen::Vector3d &current_pose,
                                   const Eigen::Vector3d &target_pose,
                                   double delta_t,
                                   double *steer) {
  const double cur_x = current_pose.x();
  const double cur_y = current_pose.y();
  const double cur_theta = current_pose.z();
  const double target_x = target_pose.x();
  const double target_y = target_pose.y();
  Eigen::Vector2d cur_heading{std::cos(cur_theta), std::sin(cur_theta)};
  Eigen::Vector2d to_target{target_x - cur_x, target_y - cur_y};
  double cross_prod = cur_heading.x() * to_target.y() - cur_heading.y() * to_target.x();
  double dot_prod = cur_heading.x() * to_target.x() + cur_heading.y() * to_target.y();
  double angle_error = std::acos(
      Clamp<double>(dot_prod / (to_target.norm() * cur_heading.norm()), -1.0, 1.0));
  angle_error = cross_prod < 0.0 ? -angle_error : angle_error;
  this->lat_error_buffer_.push_back(angle_error);
  double differential_angle_error = 0.0;
  double intergal_angle_error = 0.0;
  if (lat_error_buffer_.size() >= 2) {
    differential_angle_error = (lat_error_buffer_.back() - *(lat_error_buffer_.end() - 2)) / delta_t;
    intergal_angle_error = std::accumulate(lat_error_buffer_.begin(), lat_error_buffer_.end(), 0.0) * delta_t;
  } else {
    differential_angle_error = 0.0;
    intergal_angle_error = 0.0;
  }
  *steer = Clamp<double>(pid_configs_.lat_configs.lat_kp * angle_error
                             + pid_configs_.lat_configs.lat_kd * differential_angle_error / delta_t
                             + pid_configs_.lat_configs.lat_ki * intergal_angle_error * delta_t, -1.0, 1.0);
  return true;
}

template<class T>
T PIDController::Clamp(const T &value, const T &lower, const T &upper) {
  ROS_ASSERT(upper > lower);
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}
bool PIDController::GetMatchedPointByPosition(double x,
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

bool PIDController::GetMatchedPointByAbsoluteTime(double &time_stamp,
                                                  const planning_msgs::Trajectory &trajectory,
                                                  planning_msgs::TrajectoryPoint &matched_tp) {
  auto trajectory_time_stamp = trajectory.header.stamp;
  double relative_time = time_stamp - trajectory_time_stamp.toSec();
  return PIDController::GetMatchedPointByRelativeTime(relative_time, trajectory, matched_tp);
}

bool PIDController::GetMatchedPointByRelativeTime(double relative_time,
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

}