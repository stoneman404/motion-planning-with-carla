#include "obstacle_filter/obstacle.hpp"
#include "obstacle_filter/obstacle_filter.hpp"
#include "planning_config.hpp"
#include <memory>
#include <nav_msgs/Odometry.h>

namespace planning{
ObstacleFilter& ObstacleFilter::Instance() {
  static ObstacleFilter instance = ObstacleFilter();
  return instance;
}

void ObstacleFilter::UpdateObstacles(
    const std::unordered_map<int, derived_object_msgs::Object> &objects,
    const nav_msgs::Odometry& ego_vehicle) {
  obstacles_ = decltype(obstacles_)();
  if (objects.empty()){
    ROS_WARN("[ObstacleFilter::UpdateObstacles], the objects is empty");
    return;
  }
  double filter_radius = PlanningConfig::Instance().filter_obstacle_length();
  for (const auto& object : objects){
    double dx = object.second.pose.position.x - ego_vehicle.pose.pose.position.x;
    double dy = object.second.pose.position.y - ego_vehicle.pose.pose.position.y;
    double distance = std::hypot(dx, dy);
    if (distance > filter_radius){
      ROS_INFO("[ObstacleFilter::UpdateObstacles], "
               "the obstacle is far away from ego vehicle, ignore");
      continue;
    }
    obstacles_.emplace_back(new Obstacle(object.second));

  }
  ROS_INFO("[ObstacleFilter::UpdateObstacles], "
           "the obstacles's size after updated, %lu",
           obstacles_.size());

}
}