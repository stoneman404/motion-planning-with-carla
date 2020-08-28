#include "planning_config.hpp"

namespace planning {

PlanningConfig &PlanningConfig::Instance() {
  static PlanningConfig instance;
  return instance;
}

void PlanningConfig::UpdateParams(const ros::NodeHandle &nh) {
  nh.param<double>("/local_planner/obstacle_trajectory_time", obstacle_trajectory_time_, 8.0);
  nh.param<double>("/local_planner/delta_t", delta_t_, 0.1);
  nh.param<double>("/local_planner/filter_obstacle_length", filter_obstacle_length_, 100);
  nh.param<double>("/local_planner/collison_buffer", collision_buffer_, 5.0);
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
  nh.param<int>("/local_planner/spline_order_", spline_order_, 5);
}

void PlanningConfig::UpdateVehicleParams(const derived_object_msgs::Object &object,
                                         const carla_msgs::CarlaEgoVehicleInfo &vehicle_info) {

  this->vehicle_params_.length = object.shape.dimensions[0];
  this->vehicle_params_.width = object.shape.dimensions[1];
  this->vehicle_params_.back_rear_to_center_length = std::fabs(
      vehicle_info.wheels[3].position.x);
  this->vehicle_params_.front_rear_to_center_length = std::fabs(
      vehicle_info.wheels[0].position.x);
  this->vehicle_params_.axle_length_ = this->vehicle_params_.front_rear_to_center_length +
      this->vehicle_params_.back_rear_to_center_length;
  this->vehicle_params_.max_steer_angle_ = vehicle_info.wheels.front().max_steer_angle;
  this->vehicle_params_.min_r_ = this->vehicle_params_.axle_length_ /
      std::tan(this->vehicle_params_.max_steer_angle_);
  this->vehicle_params_.center_of_mass_[0] = vehicle_info.center_of_mass.x;
  this->vehicle_params_.center_of_mass_[1] = vehicle_info.center_of_mass.y;
  this->vehicle_params_.center_of_mass_[2] = vehicle_info.center_of_mass.z;
}
}