#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONFIG_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONFIG_HPP_
#include <ros/ros.h>
//#include "vehicle_params.hpp"
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <derived_object_msgs/Object.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
namespace planning {

class PlanningConfig {
 public:
  PlanningConfig(const PlanningConfig &other) = delete;
  static PlanningConfig &Instance();
  /**
   * @brief : update params,
   * @param nh
   */
  void UpdateParams(const ros::NodeHandle &nh);
  /**
   * @brief : update ego vehicle params,  get these params from carla ros bridge
   * @param object
   * @param vehicle_info
   */
//  void UpdateVehicleParams(const derived_object_msgs::Object &object,
//                           const carla_msgs::CarlaEgoVehicleInfo &vehicle_info);

  int loop_rate() const { return planning_loop_rate_; }
  /**
   * @brief : obstacle trajectory total time
   * @return
   */
  double obstacle_trajectory_time() const;

  /**
   *
   * @return
   */
  double delta_t() const { return delta_t_; }

  /**
   *
   * @return
   */
  const VehicleParams &vehicle_params() const;
  ////////// reference smoother params /////////////////
  /**
   *
   * @return
   */
  double reference_smoother_distance_weight() const;

  /**
   *
   * @return
   */
  double reference_smoother_curvature_weight() const;

  /**
   *
   * @return
   */
  double reference_smoother_deviation_weight() const;

  /**
   *
   * @return
   */
  double reference_smoother_heading_weight() const;

  /**
   *
   * @return
   */
  double reference_smoother_max_curvature() const;

  /**
   *
   * @return
   */
  double max_lon_acc() const;
  double min_lon_acc() const { return min_lon_jerk_; }
  double max_lon_velocity() const;
  double min_lon_velocity() const { return min_lon_velocity_; }
  double min_lon_jerk() const { return min_lon_jerk_; }
  double max_lon_jerk() const { return max_lon_jerk_; }

  /**
   *
   * @return
   */
  int spline_order() const;

  /**
   *
   * @return
   */
  double max_lookahead_time() const;

  /**
   * @brief: min lookahead time, used for frenet lattice motion_planner to sample the end conditions
   * @return
   */
  double min_lookahead_time() const;

  /**
   *
   * @return
   */
  double lon_safety_buffer() const;

  /**
   *
   * @return
   */
  double lat_safety_buffer() const;

  /**
   *
   * @return
   */
  double reference_max_forward_distance() const;

  /**
   *
   * @return
   */
  double reference_max_backward_distance() const;

  /**
   * @brief : getter, get the target speed
   * @return
   */
  double target_speed() const;

  /**
   * @brief : max lookahead distance for local motion_planner
   * @return
   */
  double max_lookahead_distance() const;

  /**
   * @brief : max lookback distance for local motion_planner, used for check the target lane is safe to change lane
   * @return
   */
  double max_lookback_distance() const;

  /**
   * @brief : min lookahead distance for local motion_planner
   * @return
   */
  double min_lookahead_distance() const;

  /**
   * @brief : the forward clear distance threshold for change lane
   * @return :
   */
  double maneuver_forward_clear_threshold() const;

  /**
   * @brief : the backward clear distance threshold for change lane, factor * target_speed
   * @return
   */
  double maneuver_backward_clear_threshold() const;

  double maneuver_target_lane_forward_clear_threshold() const;

  double maneuver_target_lane_backward_clear_threshold() const;

  /**
   * @brief : the speed discount factor for change lane
   * @return
   */
  double maneuver_change_lane_speed_discount_factor() const;

  double maneuver_execute_time_length() const;

  double maneuver_safety_cost_gain() const;

  double maneuver_efficiency_cost_gain() const;

  double maneuver_comfort_cost_gain() const;
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
//  VehicleParams vehicle_params_; // ego_vehicle's params
  double obstacle_trajectory_time_{}; // the trajectory total time of obstacles
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
  double reference_max_forward_distance_ = 400.0;
  double reference_max_backward_distance_ = 10.0;
  double max_lon_acc_ = 1.0;
  double min_lon_acc_{};
  double max_lon_velocity_ = 10.0;
  double min_lon_velocity_{};
  double target_speed_{};
  double max_lookback_distance_{};
  double maneuver_forward_clear_threshold_{};
  double maneuver_backward_clear_threshold_{};
  double maneuver_target_lane_backward_clear_threshold_{};
  double maneuver_target_lane_forward_clear_threshold_{};
  double maneuver_change_lane_speed_discount_factor_{};
  double maneuver_execute_time_length_{};
  double maneuver_safety_cost_gain_{};
  double maneuver_efficiency_cost_gain_{};
  double maneuver_comfort_cost_gain_{};
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
