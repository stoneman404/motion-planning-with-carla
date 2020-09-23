#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONFIG_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONFIG_HPP_
#include <ros/ros.h>
#include "vehicle_params.hpp"
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <derived_object_msgs/Object.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
namespace planning {

class PlanningConfig {
 public:
  PlanningConfig(const PlanningConfig &other) = delete;
  static PlanningConfig &Instance();
  void UpdateParams(const ros::NodeHandle &nh);
  void UpdateVehicleParams(const derived_object_msgs::Object &object,
                           const carla_msgs::CarlaEgoVehicleInfo &vehicle_info);
  double collision_buffer() const;
  double obstacle_trajectory_time() const;
  double delta_t() const { return delta_t_; }
  double filter_obstacle_length() const;
  const VehicleParams &vehicle_params() const;
  ////////// reference smoother params /////////////////
  double reference_smoother_distance_weight() const;
  double reference_smoother_curvature_weight() const;
  double reference_smoother_deviation_weight() const;
  double reference_smoother_heading_weight() const;
  double reference_smoother_max_curvature() const;
  double max_acc() const;
  double max_velocity() const;

  int spline_order() const;
  double max_lookahead_time() const;
  double lon_safety_buffer() const;
  double lat_safety_buffer() const;
  double reference_max_forward_distance() const;
  double reference_max_backward_distance() const;
  double target_speed() const;
  double max_lookahead_distance() const;
  double max_lookback_distance() const;
  double min_lookahead_distance() const;
  double maneuver_forward_clear_threshold() const;
  double maneuver_backward_clear_threshold() const;
  double maneuver_change_lane_speed_discount_factor() const;

 private:
  VehicleParams vehicle_params_; // ego_vehicle's params
  double obstacle_trajectory_time_{}; // the trajectory total time of obstacles
  double delta_t_{}; // the trajectory delta time
  double filter_obstacle_length_{}; // we ignore the obstacles far away this length
  double collision_buffer_{}; // the buffer to avoid collision
  double max_lookahead_distance_{}; // the max lookahead distance for ego vehicle
  double min_lookahead_distance_{};
  double max_lookahead_time_ = 8.0; // max lookahead time
  double lon_safety_buffer_ = 6.0; // lon_safety_buffer_
  double lat_safety_buffer_ = 2.0; // lat_safety_buffer_;
  double reference_smoother_distance_weight_ = 20.0;
  double reference_smoother_curvature_weight_ = 1.0;
  double reference_smoother_deviation_weight_ = 8.0;
  double reference_smoother_heading_weight_ = 50.0;
  double reference_smoother_max_curvature_ = 100;
  int spline_order_ = 3;
  double reference_max_forward_distance_ = 400.0;
  double reference_max_backward_distance_ = 10.0;
  double max_acc_ = 1.0;
  double max_velocity_ = 10.0;
  double target_speed_{};
  double max_lookback_distance_{};
  double maneuver_forward_clear_threshold_{};
  double maneuver_backward_clear_threshold_{};
  double maneuver_change_lane_speed_discount_factor_{};
 private:
  PlanningConfig() = default;
  ~PlanningConfig() = default;
};
}
#endif
