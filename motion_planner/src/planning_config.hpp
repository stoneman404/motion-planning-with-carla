#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_INCLUDE_PLANNING_CONFIG_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_INCLUDE_PLANNING_CONFIG_HPP_
#include <ros/ros.h>
#include "vehicle_state/vehicle_params.hpp"
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <derived_object_msgs/Object.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include "reference_line/reference_line.hpp"
#include "planning_msgs/Trajectory.h"
#include "planning_msgs/LateralBehaviour.h"
#include "planning_msgs/LongitudinalBehaviour.h"
#include "obstacle_manager/obstacle.hpp"

#define DEBUG false
namespace planning {

struct PlanningTarget {
  double desired_vel{};
  ReferenceLine ref_lane{};
  bool is_best_behaviour = false;
  bool has_stop_point = false;
  double stop_s{};
};

class PlanningConfig {
 public:
  static PlanningConfig &Instance();
  void UpdateParams(const ros::NodeHandle &nh);
  const std::string &planner_type() const;
  double loop_rate() const;
  double delta_t() const;
  void set_vehicle_params(const vehicle_state::VehicleParams &vehicle_params);
  const vehicle_state::VehicleParams &vehicle_params() const;
  double reference_smoother_distance_weight() const;
  double reference_smoother_deviation_weight() const;
  double reference_smoother_heading_weight() const;
  double reference_smoother_max_curvature() const;
  double reference_smoother_slack_weight() const { return reference_smoother_slack_weight_; }
  const std::string &behaviour_planner_type() const { return behaviour_planner_type_; }
  double desired_velocity() const { return desired_velocity_; }
  double sim_horizon() const { return sim_horizon_; }
  double sim_step() const { return sim_step_; }
  double safe_time_headway() const { return safe_time_headway_; }
  int acc_exponet() const { return acc_exponet_; }
  double s0() const { return s0_; }
  double s1() const { return s1_; }
  double default_lateral_approach_ratio() const { return default_lateral_approach_ratio_; }
  double cutting_in_lateral_approach_ratio() const { return cutting_in_lateral_approach_ratio_; }
  double sample_lat_threshold() const { return sample_lat_threshold_; }
  double sample_min_lon_threshold() const { return sample_min_lon_threshold_; }

  double max_lon_acc() const;
  double min_lon_acc() const;
  double max_lon_velocity() const;
  double min_lon_velocity() const;
  double min_lon_jerk() const;
  double max_lon_jerk() const;
  double max_lookahead_time() const;
  double min_lookahead_time() const;
  double lon_safety_buffer() const;
  double lat_safety_buffer() const;
  double max_lookahead_distance() const;
  double max_lookback_distance() const;
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
  double max_replan_lon_distance_threshold() const;
  double max_replan_lat_distance_threshold() const;
  int preserve_history_trajectory_point_num() const;

  double lattice_weight_centripetal_acc() const;
 private:
  vehicle_state::VehicleParams vehicle_params_{};
  std::string planner_type_;
  double planning_loop_rate_{};
  double delta_t_{}; // the trajectory delta time
  double max_lookahead_distance_{}; // the max lookahead distance for ego vehicle
  double max_lookahead_time_ = 8.0; // max lookahead time
  double min_lookahead_time_ = 1.0;
  double lon_safety_buffer_ = 4.0; // lon_safety_buffer_
  double lat_safety_buffer_ = 2.0; // lat_safety_buffer_;
  double reference_smoother_distance_weight_ = 20.0;
  double reference_smoother_curvature_weight_ = 1.0;
  double reference_smoother_deviation_weight_ = 8.0;
  double reference_smoother_heading_weight_ = 50.0;
  double reference_smoother_max_curvature_ = 100;
  double reference_smoother_slack_weight_{5.0};
  int spline_order_ = 3;
  double max_lon_acc_ = 1.0;
  double min_lon_acc_{};
  double max_lon_velocity_ = 10.0;
  double min_lon_velocity_{};
  double desired_velocity_{};
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
  double lattice_weight_centripetal_acc_{};
  double max_replan_lat_distance_threshold_{};
  double max_replan_lon_distance_threshold_{};
  int preserve_history_trajectory_point_num_{};
  std::string behaviour_planner_type_{};
  double sim_horizon_{};
  double sim_step_{};
  double safe_time_headway_{};
  int acc_exponet_{};
  double s0_{};
  double s1_{};
  double default_lateral_approach_ratio_{};
  double cutting_in_lateral_approach_ratio_{};
  double sample_lat_threshold_{};
  double sample_min_lon_threshold_{};

 private:
  PlanningConfig() = default;
  ~PlanningConfig() = default;
};
}
#endif
