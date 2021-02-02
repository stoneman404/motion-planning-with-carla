#include <pid_stanley_controller/pid_stanley_controller.hpp>
#include <numeric>
#include <math/math_utils.hpp>
namespace control {

PIDStanleyController::PIDStanleyController(const ControlConfigs &control_configs, double loop_rate)
    : control_configs_(control_configs), loop_rate_(loop_rate) {
  lon_error_buffer_.set_capacity(15);
}

bool PIDStanleyController::Execute(double current_time_stamp,
                                   const vehicle_state::VehicleState &vehicle_state,
                                   const planning_msgs::Trajectory &trajectory,
                                   carla_msgs::CarlaEgoVehicleControl &control) {
  planning_msgs::TrajectoryPoint target_tp;
  auto kinodynamic_state = vehicle_state.GetKinoDynamicVehicleState();
  double wheel_base = (vehicle_state.vehicle_params().lr_ + vehicle_state.vehicle_params().lf_);
  double front_x = kinodynamic_state.x + wheel_base * std::cos(kinodynamic_state.theta);
  double front_y = kinodynamic_state.y + wheel_base * std::sin(kinodynamic_state.theta);
  auto steer_control_state = kinodynamic_state;
  steer_control_state.x = front_x;
  steer_control_state.y = front_y;
  if (!GetMatchedPointByPosition(front_x, front_y, trajectory, target_tp)) {
    return false;
  }
//  if (cur_tp == trajectory.trajectory_points.back()) {
//    return false;
//  }


  const double cur_lon_s_dot = kinodynamic_state.v;
  const double target_lon_s_dot = std::min(target_tp.vel, (6.0 / std::fabs(target_tp.path_point.kappa) + 1e-4));
  double throttle = 0.0;
  double steer = 0.0;
  if (target_tp.path_point.s >= trajectory.trajectory_points.back().path_point.s - 1e-3) {
    control.steer = 0.0;
    control.throttle = 0.0;
    control.brake = 1.0;
    control.hand_brake = false;
    control.manual_gear_shift = false;
    return true;
  }
  if (!LongitudinalControl(cur_lon_s_dot, target_lon_s_dot, 1.0 / loop_rate_, &throttle)) {
    return false;
  }
//  double wheel_base = vehicle_state.vehicle_params().lf_ + vehicle_state.vehicle_params().lr_;
  double max_steer = vehicle_state.vehicle_params().max_steer_angle_;
  std::cout << "max_steer: " << max_steer << std::endl;
  if (!StanleySteerControl(max_steer, wheel_base, steer_control_state, target_tp, trajectory, &steer)) {
    return false;
  }
  control.steer = steer;
  control.throttle = throttle;
  control.brake = 0.0;
  control.hand_brake = false;
  control.manual_gear_shift = false;
  return true;
}

bool PIDStanleyController::LongitudinalControl(double current_speed, double target_speed,
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

template<class T>
T PIDStanleyController::Clamp(const T &value, const T &lower, const T &upper) {
  ROS_ASSERT(upper > lower);
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

bool PIDStanleyController::GetMatchedPointByPosition(double x, double y,
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

bool PIDStanleyController::GetMatchedPointByAbsoluteTime(double &time_stamp,
                                                         const planning_msgs::Trajectory &trajectory,
                                                         planning_msgs::TrajectoryPoint &matched_tp) {
  auto trajectory_time_stamp = trajectory.header.stamp;
  double relative_time = time_stamp - trajectory_time_stamp.toSec();
  return PIDStanleyController::GetMatchedPointByRelativeTime(relative_time, trajectory, matched_tp);
}

bool PIDStanleyController::GetMatchedPointByRelativeTime(double relative_time,
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

bool PIDStanleyController::GetMatchedPointByS(double s,
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

bool PIDStanleyController::StanleySteerControl(double max_steer,
                                               double wheelbase,
                                               const vehicle_state::KinoDynamicState &vehicle_state,
                                               const planning_msgs::TrajectoryPoint &target_tp,
                                               const planning_msgs::Trajectory &trajectory,
                                               double *steer) {

  Eigen::Vector2d target_to_ego{vehicle_state.x - target_tp.path_point.x, vehicle_state.y - target_tp.path_point.y};
  Eigen::Vector2d ego_heading{std::cos(vehicle_state.theta), std::sin(vehicle_state.theta)};
  double cross_prod = target_to_ego.x() * ego_heading.y() - target_to_ego.y() * ego_heading.x();
//  double cross_error = std::hypot(target_tp.path_point.x - vehicle_state.x,
//                                  target_tp.path_point.y - vehicle_state.y);
  double cross_error = std::fabs(cross_prod);
  double theta_error = common::MathUtils::NormalizeAngle(target_tp.path_point.theta - vehicle_state.theta);
  double k = 0.3;
  if (std::fabs(vehicle_state.v) < 0.5) {
    *steer = 0.0;
    return true;
  }
  double theta_d = std::atan2(k * cross_error, vehicle_state.v);
  double calc_steer = 1 * theta_error + theta_d;
  std::cout << " calc_steer : " << calc_steer << std::endl;
  double normalized_steer = (calc_steer) / max_steer;

  *steer = Clamp<double>(normalized_steer, -1.0, 1.0);
  return true;
}

}