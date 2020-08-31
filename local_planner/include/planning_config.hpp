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
    double collision_buffer() const { return collision_buffer_; }
    double obstacle_trajectory_time() const { return obstacle_trajectory_time_; }
    double delta_t() const { return delta_t_; }
    double filter_obstacle_length() const { return filter_obstacle_length_; }
    double max_lookahead_distance() const { return max_lookahead_distance_; }
    const VehicleParams &vehicle_params() const { return vehicle_params_; }
    ////////// reference smoother params /////////////////
    double reference_smoother_distance_weight() const {
      return reference_smoother_distance_weight_;
    }
    double reference_smoother_curvature_weight() const {
      return reference_smoother_curvature_weight_;
    }
    double reference_smoother_deviation_weight() const {
      return reference_smoother_deviation_weight_;
    }
    double reference_smoother_heading_weight() const {
      return reference_smoother_heading_weight_;
    }
    double reference_smoother_max_curvature() const {
      return reference_smoother_max_curvature_;
    }

    int spline_order() const { return spline_order_; }
    double max_lookahead_time() const {return max_lookahead_time_;}
    double safety_buffer() const {return safety_buffer_;}
private:
    VehicleParams vehicle_params_; // ego_vehicle's params
    double obstacle_trajectory_time_; // the trajectory total time of obstacles
    double delta_t_; // the trajectory delta time
    double filter_obstacle_length_; // we ignore the obstacles far away this length
    double collision_buffer_; // the buffer to avoid collision
    double max_lookahead_distance_; // the max lookahead distance for ego vehicle
    double max_lookahead_time_ = 8.0; // max lookahead time
    double safety_buffer_ = 6.0; // safety_buffer_
    double reference_smoother_distance_weight_ = 20.0;
    double reference_smoother_curvature_weight_ = 1.0;
    double reference_smoother_deviation_weight_ = 8.0;
    double reference_smoother_heading_weight_ = 50.0;
    double reference_smoother_max_curvature_ = 100;
    int spline_order_ = 3;

private:
    PlanningConfig() = default;
    ~PlanningConfig() = default;
};
}
#endif
