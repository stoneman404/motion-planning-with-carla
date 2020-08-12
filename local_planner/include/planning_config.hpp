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
    const VehicleParams& vehicle_params() const {return vehicle_params_;}
    double obstacle_trajectory_time() const { return obstacle_trajectory_time_; }
    double delta_t() const { return delta_t_; }
    double filter_obstacle_length() const {return filter_obstacle_length_;}
    void UpdateVehicleParams(const derived_object_msgs::Object& object,
        const carla_msgs::CarlaEgoVehicleInfo& vehicle_info);
private:
    VehicleParams vehicle_params_;
    double obstacle_trajectory_time_;
    double delta_t_;
    double filter_obstacle_length_;

private:
    PlanningConfig() = default;
    ~PlanningConfig() = default;
};
}
#endif
