#include "obstacle_filter/obstacle.hpp"
#include "obstacle_filter/obstacle_filter.hpp"
#include <memory>
#include <nav_msgs/Odometry.h>

namespace planning{

ObstacleFilter& ObstacleFilter::Instance() {
  static ObstacleFilter instance = ObstacleFilter();
  return instance;
}

void ObstacleFilter::AddObstacle(const std::shared_ptr<Obstacle> &obstacle_ptr) {
  obstacles_.emplace(obstacle_ptr->Id(), obstacle_ptr);
}

void ObstacleFilter::UpdateObstacles(const std::list<std::shared_ptr<Obstacle>> &obstacles) {
  obstacles_.clear();
  for (const auto& obstacle : obstacles){
    if (obstacle->road_id() != VehicleState::Instance().road_id()){
      continue;
    }
    obstacles_.emplace(obstacle->Id(), obstacle);
  }
}

}