#include "planning_config.hpp"

namespace planning {

PlanningConfig &PlanningConfig::Instance() {
  static PlanningConfig instance;
  return instance;
}

void PlanningConfig::UpdateParams(const ros::NodeHandle &nh) {
  nh.param<double>("/local_planner/obstacle_trajectory_time",
                   obstacle_trajectory_time_, 8.0);
  nh.param<double>("/local_planner/delta_t", delta_t_, 0.1);
//  nh.param<double>("/local_planner/filter_obstacle_length", filter_obstacle_length_, 100);
//  nh.param<double>("/local_planner/collison_buffer", collision_buffer_, 1.0);
  nh.param<double>("/local_planner/reference_smoother_deviation_weight",
                   reference_smoother_deviation_weight_, 10.0);
  nh.param<double>("/local_planner/reference_smoother_curvature_weight",
                   reference_smoother_curvature_weight_, 4.0);
  nh.param<double>("/local_planner/reference_smoother_heading_weight",
                   reference_smoother_heading_weight_, 10.0);
  nh.param<double>("/local_planner/reference_smoother_distance_weight",
                   reference_smoother_distance_weight_, 5.0);
  nh.param<double>("/local_planner/reference_smoother_max_curvature",
                   reference_smoother_max_curvature_, 100);
  nh.param<int>("/local_planner/spline_order_", spline_order_, 3);
  nh.param<double>("/local_planner/max_lookahead_time",
                   max_lookahead_time_, 8.0);
  nh.param<double>("/local_planner/min_lookahead_time", min_lookahead_time_, 1.0);
  nh.param<double>("/local_planner/lon_safety_buffer",
                   lon_safety_buffer_, 3.0);
  nh.param<double>("/local_planner/lat_safety_buffer",
                   lat_safety_buffer_, 1.0);
  nh.param<double>("/local_planner/max_lookahead_distance",
                   max_lookahead_distance_, 100);
  nh.param<double>("/local_planner/max_lookback_distance",
                   max_lookback_distance_, 30.0);
  nh.param<double>("/local_planner/reference_max_forward_distance",
                   reference_max_forward_distance_, 400.0);
  nh.param<double>("/local_planner/reference_max_backward_distance",
                   reference_max_backward_distance_, 60.0);
  nh.param<double>("/local_planner/max_acc", max_acc_, 2.0);
  nh.param<double>("/local_planner/max_velocity", max_velocity_, 10.0);
  nh.param<double>("/local_planner/target_speed", target_speed_, 8.333);
  nh.param<double>("/local_planner/maneuver_forward_clear_threshold",
                   maneuver_forward_clear_threshold_, 20);
  nh.param<double>("/local_planner/maneuver_backward_clear_threshold",
                   maneuver_backward_clear_threshold_, 10);
  nh.param<double>("/local_planner/maneuver_target_lane_forward_clear_distance",
                   maneuver_forward_clear_threshold_, 20);
  nh.param<double>("/local_planner/maneuver_target_lane_backward_clear_distance",
                   maneuver_target_lane_backward_clear_threshold_, 15);
  nh.param<double>("/local_planner/min_lookahead_distance",
                   min_lookahead_distance_, 1.0);
  nh.param<double>("/local_planner/maneuver_change_lane_speed_discount_factor",
                   maneuver_change_lane_speed_discount_factor_, 0.6);
  nh.param<double>("/local_planner/maneuver_execute_time_length",
                   maneuver_execute_time_length_, 8.0);
  nh.param<double>("/local_planner/maneuver_safety_cost_gain",
                   maneuver_safety_cost_gain_, 10.0);
  nh.param<double>("/local_planner/maneuver_efficiency_cost_gain",
                   maneuver_efficiency_cost_gain_, 6.0);
  nh.param<double>("/local_planner/maneuver_comfort_cost_gain",
                   maneuver_comfort_cost_gain_, 5.0);
}

void PlanningConfig::UpdateVehicleParams(const derived_object_msgs::Object &object,
                                         const carla_msgs::CarlaEgoVehicleInfo &vehicle_info) {

  this->vehicle_params_.length = object.shape.dimensions[0];
  this->vehicle_params_.width = object.shape.dimensions[1];
  this->vehicle_params_.back_axle_to_center_length = std::fabs(
      vehicle_info.wheels[3].position.x);
  this->vehicle_params_.front_axle_to_center_length = std::fabs(
      vehicle_info.wheels[0].position.x);
  this->vehicle_params_.axle_length_ = this->vehicle_params_.front_axle_to_center_length +
      this->vehicle_params_.back_axle_to_center_length;
  this->vehicle_params_.max_steer_angle_ = vehicle_info.wheels.front().max_steer_angle;
  this->vehicle_params_.min_r_ = this->vehicle_params_.axle_length_ /
      std::tan(this->vehicle_params_.max_steer_angle_);
  this->vehicle_params_.center_of_mass_[0] = vehicle_info.center_of_mass.x;
  this->vehicle_params_.center_of_mass_[1] = vehicle_info.center_of_mass.y;
  this->vehicle_params_.center_of_mass_[2] = vehicle_info.center_of_mass.z;
}

double PlanningConfig::obstacle_trajectory_time() const { return obstacle_trajectory_time_; }

double PlanningConfig::max_lookahead_distance() const { return max_lookahead_distance_; }

const VehicleParams &PlanningConfig::vehicle_params() const { return vehicle_params_; }

double PlanningConfig::reference_smoother_distance_weight() const {
  return reference_smoother_distance_weight_;
}

double PlanningConfig::reference_smoother_curvature_weight() const {
  return reference_smoother_curvature_weight_;
}

double PlanningConfig::reference_smoother_deviation_weight() const {
  return reference_smoother_deviation_weight_;
}

double PlanningConfig::reference_smoother_max_curvature() const {
  return reference_smoother_max_curvature_;
}

double PlanningConfig::reference_smoother_heading_weight() const {
  return reference_smoother_heading_weight_;
}

double PlanningConfig::max_acc() const { return max_acc_; }

int PlanningConfig::spline_order() const { return spline_order_; }

double PlanningConfig::max_lookahead_time() const { return max_lookahead_time_; }

double PlanningConfig::lon_safety_buffer() const { return lon_safety_buffer_; }

double PlanningConfig::lat_safety_buffer() const { return lat_safety_buffer_; }

double PlanningConfig::reference_max_backward_distance() const { return reference_max_backward_distance_; }

double PlanningConfig::reference_max_forward_distance() const { return reference_max_forward_distance_; }

double PlanningConfig::max_velocity() const { return max_velocity_; }

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

}