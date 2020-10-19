#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_OBSTACLE_FILTER_OBSTACLE_FILTER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_OBSTACLE_FILTER_OBSTACLE_FILTER_HPP_

#include <carla_msgs/CarlaActorList.h>
#include <planning_msgs/Trajectory.h>
#include <carla_waypoint_types/GetActorWaypoint.h>
#include <derived_object_msgs/ObjectArray.h>
#include <ros/node_handle.h>
#include <memory>
#include <list>
#include <unordered_map>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <vehicle_state/vehicle_state.hpp>
#include "obstacle.hpp"

namespace planning {
// we only process walkers and vehicles
class ObstacleFilter {
 public:
  static ObstacleFilter &Instance();
  ObstacleFilter(const ObstacleFilter &other) = default;
  void UpdateObstacles(const std::list<std::shared_ptr<Obstacle>> &objects);
  void AddObstacle(const std::shared_ptr<Obstacle> &obstacle_ptr);
  size_t ObstaclesSize() const { return obstacles_.size(); }
  const std::unordered_map<int, std::shared_ptr<Obstacle>> &Obstacles() const { return obstacles_; }
  std::unordered_map<int, std::shared_ptr<Obstacle>> &multable_obstacles() { return obstacles_; }
 private:
  std::unordered_map<int, std::shared_ptr<Obstacle>> obstacles_;
 private:
  ObstacleFilter() = default;
  ~ObstacleFilter() = default;
};

}

#endif
