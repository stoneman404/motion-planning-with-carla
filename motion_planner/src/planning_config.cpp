#include <vehicle_state/vehicle_params.hpp>
#include "planning_config.hpp"

namespace planning {

PlanningConfig &PlanningConfig::Instance() {
  static PlanningConfig instance;
  return instance;
}

void PlanningConfig::UpdateParams(const ros::NodeHandle &nh) {
  nh.param<std::string>("/motion_planner/planner_type", planner_type_, "frenet_lattice");
  nh.param<double>("/motion_planner/loop_rate", planning_loop_rate_, 8.0);
  nh.param<double>("/motion_planner/delta_t", delta_t_, 0.1);
  nh.param<double>("/motion_planner/reference_smoother_deviation_weight", reference_smoother_deviation_weight_, 5.5);
  nh.param<double>("/motion_planner/reference_smoother_curvature_weight", reference_smoother_curvature_weight_, 4.0);
  nh.param<double>("/motion_planner/reference_smoother_heading_weight", reference_smoother_heading_weight_, 9.5);
  nh.param<double>("/motion_planner/reference_smoother_distance_weight", reference_smoother_distance_weight_, 6);
  nh.param<double>("/motion_planner/reference_smoother_max_curvature", reference_smoother_max_curvature_, 6);
  nh.param<int>("/motion_planner/spline_order", spline_order_, 3);
  nh.param<double>("/motion_planner/max_lookahead_time", max_lookahead_time_, 8.0);
  nh.param<double>("/motion_planner/min_lookahead_time", min_lookahead_time_, 1.0);
  nh.param<double>("/motion_planner/lon_safety_buffer", lon_safety_buffer_, 2.0);
  nh.param<double>("/motion_planner/lat_safety_buffer", lat_safety_buffer_, .2);
  nh.param<double>("/motion_planner/max_lookahead_distance", max_lookahead_distance_, 80);
  nh.param<double>("/motion_planner/max_lookback_distance", max_lookback_distance_, 30.0);
  nh.param<double>("/motion_planner/max_lon_acc", max_lon_acc_, 6.0);
  nh.param<double>("/motion_planner/min_lon_acc", min_lon_acc_, -6.0);
  nh.param<double>("/motion_planner/max_lon_velocity", max_lon_velocity_, 20.0);
  nh.param<double>("/motion_planner/min_lon_velocity", min_lon_velocity_, -.1);
  nh.param<double>("/motion_planner/max_lon_jerk", max_lon_jerk_, 10.0);
  nh.param<double>("/motion_planner/min_lon_jerk", min_lon_jerk_, -10.0);
  nh.param<double>("/motion_planner/min_kappa", min_kappa_, -2.0);
  nh.param<double>("/motion_planner/max_kappa", max_kappa_, 2.0);
  nh.param<double>("/motion_planner/min_lat_acc", min_lat_acc_, -4.0);
  nh.param<double>("/motion_planner/max_lat_acc", max_lat_acc_, 4.0);
  nh.param<double>("/motion_planner/desired_velocity", desired_velocity_, 8.333);
  nh.param<double>("/motion_planner/lattice_weight_opposite_side_offset",
                   lattice_weight_opposite_side_offset_,10.0);
  nh.param<double>("/motion_planner/lattice_weight_same_side_offset",
                   lattice_weight_same_side_offset_,1.0);
  nh.param<double>("/motion_planner/lattice_weight_dist_travelled",
                   lattice_weight_dist_travelled_,10.0);
  nh.param<double>("/motion_planner/lattice_weight_target_speed", lattice_weight_target_speed_, 2.0);
  nh.param<double>("/motion_planner/lattice_weight_collision", lattice_weight_collision_, 5);
  nh.param<double>("/motion_planner/lattice_weight_lon_jerk", lattice_weight_lon_jerk_, 10.0);
  nh.param<double>("/motion_planner/lattice_weight_lon_target", lattice_weight_lon_target_, 20.0);
  nh.param<double>("/motion_planner/lattice_weight_lat_offset", lattice_weight_lat_offset_, 10.0);
  nh.param<double>("/motion_planner/lattice_weight_lat_jerk", lattice_weight_lat_jerk_, 1);
  nh.param<double>("/motion_planner/lattice_weight_centripetal_acc", lattice_weight_centripetal_acc_, 1.5);
  nh.param<double>("/motion_planner/max_replan_lat_distance_threshold", max_replan_lat_distance_threshold_, 0.5);
  nh.param<double>("/motion_planner/max_replan_lon_distance_threshold", max_replan_lon_distance_threshold_, 2.5);
  nh.param<int>("/motion_planner/preserve_history_trajectory_point_num", preserve_history_trajectory_point_num_, 15);
  nh.param<std::string> ("/motion_planner/behaviour_planner_type", behaviour_planner_type_, "mpdm");
  nh.param<double>("/motion_planner/sim_horizon", sim_horizon_, 10.0);
  nh.param<double>("/motion_planner/sim_step", sim_step_, 0.25);
  nh.param<double>("/motion_planner/safe_time_headway", safe_time_headway_, 0.5);
  nh.param<int>("/motion_planner/acc_exponet", acc_exponet_, 4);
  nh.param<double>("/motion_planner/s0", s0_, 2.0);
  nh.param<double>("/motion_planner/s1", s1_, 0.0);
  nh.param<double>("/motion_planner/default_lateral_approach_ratio", default_lateral_approach_ratio_, 0.995);
  nh.param<double>("/motion_planner/cutting_in_lateral_approach_ratio", cutting_in_lateral_approach_ratio_, 0.95);
  nh.param<double>("/motion_planner/sample_lat_threshold", sample_lat_threshold_, 6.0);
  nh.param<double>("/motion_planner/sample_min_lon_threshold", sample_min_lon_threshold_, 20.0);
}
const std::string &PlanningConfig::planner_type() const { return planner_type_; }
double PlanningConfig::max_lookahead_distance() const { return max_lookahead_distance_; }
double PlanningConfig::reference_smoother_distance_weight() const { return reference_smoother_distance_weight_; }
double PlanningConfig::reference_smoother_deviation_weight() const {
  return reference_smoother_deviation_weight_;
}
double PlanningConfig::reference_smoother_max_curvature() const {
  return reference_smoother_max_curvature_;
}
double PlanningConfig::reference_smoother_heading_weight() const {
  return reference_smoother_heading_weight_;
}
double PlanningConfig::max_lon_acc() const { return max_lon_acc_; }
double PlanningConfig::max_lookahead_time() const { return max_lookahead_time_; }
double PlanningConfig::lon_safety_buffer() const { return lon_safety_buffer_; }
double PlanningConfig::lat_safety_buffer() const { return lat_safety_buffer_; }
double PlanningConfig::max_lon_velocity() const { return max_lon_velocity_; }
double PlanningConfig::max_lookback_distance() const { return max_lookback_distance_; }
double PlanningConfig::min_lookahead_time() const { return min_lookahead_time_; }
double PlanningConfig::lattice_weight_opposite_side_offset() const {
  return lattice_weight_opposite_side_offset_;
}
double PlanningConfig::lattice_weight_same_side_offset() const {
  return lattice_weight_same_side_offset_;
}
double PlanningConfig::lattice_weight_dist_travelled() const {
  return lattice_weight_dist_travelled_;
}
double PlanningConfig::lattice_weight_target_speed() const {
  return lattice_weight_target_speed_;
}
double PlanningConfig::lattice_weight_collision() const {
  return lattice_weight_collision_;
}
double PlanningConfig::lattice_weight_lon_jerk() const {
  return lattice_weight_lon_jerk_;
}
double PlanningConfig::lattice_weight_lon_target() const {
  return lattice_weight_lon_target_;
}
double PlanningConfig::lattice_weight_lat_jerk() const {
  return lattice_weight_lat_jerk_;
}
double PlanningConfig::lattice_weight_lat_offset() const {
  return lattice_weight_lat_offset_;
}
double PlanningConfig::min_kappa() const {
  return min_kappa_;
}
double PlanningConfig::max_kappa() const {
  return max_kappa_;
}
double PlanningConfig::min_lat_acc() const { return min_lat_acc_; }
double PlanningConfig::max_lat_acc() const { return max_lat_acc_; }
double PlanningConfig::loop_rate() const { return planning_loop_rate_; }
double PlanningConfig::delta_t() const { return delta_t_; }
double PlanningConfig::min_lon_acc() const { return min_lon_acc_; }
double PlanningConfig::min_lon_velocity() const { return min_lon_velocity_; }
double PlanningConfig::min_lon_jerk() const { return min_lon_jerk_; }
double PlanningConfig::max_lon_jerk() const { return max_lon_jerk_; }
const vehicle_state::VehicleParams &PlanningConfig::vehicle_params() const {
  return vehicle_params_;
}

void PlanningConfig::set_vehicle_params(const vehicle_state::VehicleParams &vehicle_params) {
  this->vehicle_params_ = vehicle_params;
}

double PlanningConfig::max_replan_lat_distance_threshold() const {
  return max_replan_lat_distance_threshold_;
}
double PlanningConfig::max_replan_lon_distance_threshold() const {
  return max_replan_lon_distance_threshold_;
}
int PlanningConfig::preserve_history_trajectory_point_num() const { return preserve_history_trajectory_point_num_; }
double PlanningConfig::lattice_weight_centripetal_acc() const {
  return lattice_weight_centripetal_acc_;
}
}