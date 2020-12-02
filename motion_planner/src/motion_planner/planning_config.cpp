#include <common/vehicle_params.hpp>
#include "motion_planner/planning_config.hpp"

namespace planning {

PlanningConfig &PlanningConfig::Instance() {
  static PlanningConfig instance;
  return instance;
}

void PlanningConfig::UpdateParams(const ros::NodeHandle &nh) {
  nh.param<int>("/behaviour_planner/motion_planning/loop_rate", planning_loop_rate_, 10);
  nh.param<double>("/behaviour_planner/motion_planning/obstacle_trajectory_time",
                   obstacle_trajectory_time_, 8.0);
  ROS_INFO("obstacle_trajectory_time: %f", obstacle_trajectory_time_);
  nh.param<double>("/behaviour_planner/motion_planning/delta_t", delta_t_, 0.1);
  nh.param<double>("/behaviour_planner/motion_planning/reference_smoother_deviation_weight",
                   reference_smoother_deviation_weight_, 5.0);
  nh.param<double>("/behaviour_planner/motion_planning/reference_smoother_curvature_weight",
                   reference_smoother_curvature_weight_, 20.0);
  nh.param<double>("/behaviour_planner/motion_planning/reference_smoother_heading_weight",
                   reference_smoother_heading_weight_, 6.0);
  nh.param<double>("/behaviour_planner/motion_planning/reference_smoother_distance_weight",
                   reference_smoother_distance_weight_, 4);
  nh.param<double>("/behaviour_planner/motion_planning/reference_smoother_max_curvature",
                   reference_smoother_max_curvature_, 5);
  nh.param<int>("/behaviour_planner/motion_planning/spline_order", spline_order_, 3);
  ROS_INFO("spline_order_: %i", spline_order_);
  nh.param<double>("/behaviour_planner/motion_planning/max_lookahead_time",
                   max_lookahead_time_, 8.0);
  nh.param<double>("/behaviour_planner/motion_planning/min_lookahead_time", min_lookahead_time_, 1.0);
  nh.param<double>("/behaviour_planner/motion_planning/lon_safety_buffer",
                   lon_safety_buffer_, 3.0);
  nh.param<double>("/behaviour_planner/motion_planning/lat_safety_buffer",
                   lat_safety_buffer_, 1.0);
  nh.param<double>("/behaviour_planner/motion_planning/max_lookahead_distance",
                   max_lookahead_distance_, 100);
  nh.param<double>("/behaviour_planner/motion_planning/max_lookback_distance",
                   max_lookback_distance_, 30.0);
  nh.param<double>("/behaviour_planner/motion_planning/reference_max_forward_distance",
                   reference_max_forward_distance_, 200.0);
  nh.param<double>("/behaviour_planner/motion_planning/reference_max_backward_distance",
                   reference_max_backward_distance_, 30.0);
  nh.param<double>("/behaviour_planner/motion_planning/max_lon_acc", max_lon_acc_, 2.0);
  nh.param<double>("/behaviour_planner/motion_planning/min_lon_acc", min_lon_acc_, -2.0);
  nh.param<double>("/behaviour_planner/motion_planning/max_lon_velocity", max_lon_velocity_, 10.0);
  nh.param<double>("/behaviour_planner/motion_planning/min_lon_velocity", min_lon_velocity_, 0.0);
  nh.param<double>("/behaviour_planner/motion_planning/max_lon_jerk", max_lon_jerk_, 5.0);
  nh.param<double>("/behaviour_planner/motion_planning/min_lon_jerk", min_lon_jerk_, -5.0);
  nh.param<double>("/behaviour_planner/motion_planning/target_speed", target_speed_, 8.333);
  nh.param<double>("/behaviour_planner/motion_planning/maneuver_forward_clear_threshold",
                   maneuver_forward_clear_threshold_, 20);
  nh.param<double>("/behaviour_planner/motion_planning/maneuver_backward_clear_threshold",
                   maneuver_backward_clear_threshold_, 10);
  nh.param<double>("/behaviour_planner/motion_planning/maneuver_target_lane_forward_clear_distance",
                   maneuver_forward_clear_threshold_, 20);
  nh.param<double>("/behaviour_planner/motion_planning/maneuver_target_lane_backward_clear_distance",
                   maneuver_target_lane_backward_clear_threshold_, 15);
  nh.param<double>("/behaviour_planner/motion_planning/min_lookahead_distance",
                   min_lookahead_distance_, 1.0);
  nh.param<double>("/behaviour_planner/motion_planning/maneuver_change_lane_speed_discount_factor",
                   maneuver_change_lane_speed_discount_factor_, 0.6);
  nh.param<double>("/behaviour_planner/motion_planning/maneuver_execute_time_length",
                   maneuver_execute_time_length_, 8.0);
  nh.param<double>("/behaviour_planner/motion_planning/maneuver_safety_cost_gain",
                   maneuver_safety_cost_gain_, 10.0);
  nh.param<double>("/behaviour_planner/motion_planning/maneuver_efficiency_cost_gain",
                   maneuver_efficiency_cost_gain_, 6.0);
  nh.param<double>("/behaviour_planner/motion_planning/maneuver_comfort_cost_gain",
                   maneuver_comfort_cost_gain_, 5.0);
  nh.param<double>("/behaviour_planner/motion_planning/lattice_weight_opposite_side_offset",
                   lattice_weight_opposite_side_offset_, 10.0);
  nh.param<double>("/behaviour_planner/motion_planning/lattice_weight_same_side_offset",
                   lattice_weight_same_side_offset_, 5.0);
  nh.param<double>("/behaviour_planner/motion_planning/lattice_weight_dist_travelled",
                   lattice_weight_dist_travelled_, 15.0);
  nh.param<double>("/behaviour_planner/motion_planning/lattice_weight_target_speed",
                   lattice_weight_target_speed_, 12.0);
  nh.param<double>("/behaviour_planner/motion_planning/lattice_weight_collision",
                   lattice_weight_collision_, 20);
  nh.param<double>("/behaviour_planner/motion_planning/lattice_weight_lon_jerk",
                   lattice_weight_lon_jerk_, 10.0);
  nh.param<double>("/behaviour_planner/motion_planning/lattice_weight_lon_target",
                   lattice_weight_lon_target_, 20.0);
  nh.param<double>("/behaviour_planner/motion_planning/lattice_weight_lat_offset",
                   lattice_weight_lat_offset_, 20.0);
  nh.param<double>("/behaviour_planner/motion_planning/lattice_weight_lat_jerk",
                   lattice_weight_lat_jerk_, 30);
  nh.param<double>("/behaviour_planner/motion_planning/min_kappa", min_kappa_, -10.0);
  nh.param<double>("/behaviour_planner/motion_planning/max_kappa", max_kappa_, 10.0);
  nh.param<double>("/behaviour_planner/motion_planning/min_lat_acc", min_lat_acc_, -2.0);
  nh.param<double>("/behaviour_planner/motion_planning/max_lat_acc", max_lat_acc_, 2.0);
}



double PlanningConfig::obstacle_trajectory_time() const { return obstacle_trajectory_time_; }

double PlanningConfig::max_lookahead_distance() const { return max_lookahead_distance_; }

//const thread_pool::VehicleParams &PlanningConfig::vehicle_params() const { return vehicle_params_; }

double PlanningConfig::reference_smoother_distance_weight() const { return reference_smoother_distance_weight_; }

double PlanningConfig::reference_smoother_curvature_weight() const { return reference_smoother_curvature_weight_; }

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

int PlanningConfig::spline_order() const { return spline_order_; }

double PlanningConfig::max_lookahead_time() const { return max_lookahead_time_; }

double PlanningConfig::lon_safety_buffer() const { return lon_safety_buffer_; }

double PlanningConfig::lat_safety_buffer() const { return lat_safety_buffer_; }

double PlanningConfig::reference_max_backward_distance() const { return reference_max_backward_distance_; }

double PlanningConfig::reference_max_forward_distance() const { return reference_max_forward_distance_; }

double PlanningConfig::max_lon_velocity() const { return max_lon_velocity_; }

double PlanningConfig::target_speed() const { return target_speed_; }

double PlanningConfig::min_lookahead_distance() const { return min_lookahead_distance_; }

double PlanningConfig::max_lookback_distance() const { return max_lookback_distance_; }

double PlanningConfig::maneuver_forward_clear_threshold() const {
  return maneuver_forward_clear_threshold_;
}

double PlanningConfig::maneuver_backward_clear_threshold() const {
  return maneuver_backward_clear_threshold_;
}

double PlanningConfig::maneuver_change_lane_speed_discount_factor() const {
  return maneuver_change_lane_speed_discount_factor_;
}

double PlanningConfig::maneuver_target_lane_forward_clear_threshold() const {
  return maneuver_target_lane_forward_clear_threshold_;
}

double PlanningConfig::maneuver_target_lane_backward_clear_threshold() const {
  return maneuver_target_lane_backward_clear_threshold_;
}

double PlanningConfig::maneuver_execute_time_length() const {
  return maneuver_execute_time_length_;
}

double PlanningConfig::maneuver_safety_cost_gain() const {
  return maneuver_safety_cost_gain_;
}

double PlanningConfig::maneuver_efficiency_cost_gain() const {
  return maneuver_efficiency_cost_gain_;
}

double PlanningConfig::maneuver_comfort_cost_gain() const {
  return maneuver_comfort_cost_gain_;
}

double PlanningConfig::min_lookahead_time() const { return min_lookahead_distance_; }

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

}