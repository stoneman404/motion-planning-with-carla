#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_INCLUDE_PLANNING_CONFIG_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_INCLUDE_PLANNING_CONFIG_HPP_
#include <ros/ros.h>
#include "vehicle_state/vehicle_params.hpp"
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <derived_object_msgs/Object.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>

namespace planning {

class PlanningConfig {
 public:
  PlanningConfig(const PlanningConfig &other) = delete;
  static PlanningConfig &Instance();
  void UpdateParams(const ros::NodeHandle &nh);
  int loop_rate() const { return planning_loop_rate_; }
  double delta_t() const { return delta_t_; }
  double reference_smoother_distance_weight() const;
  double reference_smoother_curvature_weight() const;
  double reference_smoother_deviation_weight() const;
  double reference_smoother_heading_weight() const;
  double reference_smoother_max_curvature() const;
  double max_lon_acc() const;
  double min_lon_acc() const { return min_lon_jerk_; }
  double max_lon_velocity() const;
  double min_lon_velocity() const { return min_lon_velocity_; }
  double min_lon_jerk() const { return min_lon_jerk_; }
  double max_lon_jerk() const { return max_lon_jerk_; }
  int spline_order() const;
  double max_lookahead_time() const;
  double min_lookahead_time() const;
  double lon_safety_buffer() const;
  double lat_safety_buffer() const;
  double target_speed() const;
  double max_lookahead_distance() const;
  double max_lookback_distance() const;
  double min_lookahead_distance() const;
  double lattice_weight_opposite_side_offset() const;
  double lattice_weight_same_side_offset() const;
  double lattice_weight_dist_travelled() const;
  double lattice_weight_target_speed() const;
  double lattice_weight_collision() const;
  double lattice_weight_lon_jerk() const;
  double lattice_weight_lon_target() const;
  double lattice_weight_lat_jerk() const;
  double lattice_weight_lat_offset() const;
  double min_kappa() const;
  double max_kappa() const;
  double min_lat_acc() const;
  double max_lat_acc() const;

 private:
  int planning_loop_rate_{};
  double delta_t_{}; // the trajectory delta time
  double max_lookahead_distance_{}; // the max lookahead distance for ego vehicle
  double min_lookahead_distance_{};
  double max_lookahead_time_ = 8.0; // max lookahead time
  double min_lookahead_time_ = 1.0;
  double lon_safety_buffer_ = 6.0; // lon_safety_buffer_
  double lat_safety_buffer_ = 2.0; // lat_safety_buffer_;
  double reference_smoother_distance_weight_ = 20.0;
  double reference_smoother_curvature_weight_ = 1.0;
  double reference_smoother_deviation_weight_ = 8.0;
  double reference_smoother_heading_weight_ = 50.0;
  double reference_smoother_max_curvature_ = 100;
  int spline_order_ = 3;
  double max_lon_acc_ = 1.0;
  double min_lon_acc_{};
  double max_lon_velocity_ = 10.0;
  double min_lon_velocity_{};
  double target_speed_{};
  double max_lookback_distance_{};
  double min_lon_jerk_{};
  double max_lon_jerk_{};
  double min_kappa_{};
  double max_kappa_{};
  double min_lat_acc_{};
  double max_lat_acc_{};
  double lattice_weight_opposite_side_offset_{};
  double lattice_weight_same_side_offset_{};
  double lattice_weight_dist_travelled_{};
  double lattice_weight_target_speed_{};
  double lattice_weight_collision_{};
  double lattice_weight_lon_jerk_{};
  double lattice_weight_lon_target_{};
  double lattice_weight_lat_jerk_{};
  double lattice_weight_lat_offset_{};

 private:
  PlanningConfig() = default;
  ~PlanningConfig() = default;
};
}
#endif
