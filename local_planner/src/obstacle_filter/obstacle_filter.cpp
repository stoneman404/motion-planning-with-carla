#include "obstacle_filter/obstacle.hpp"
#include "obstacle_filter/obstacle_filter.hpp"
#include <memory>
#include <nav_msgs/Odometry.h>

namespace planning {

ObstacleFilter &ObstacleFilter::Instance() {
  static ObstacleFilter instance = ObstacleFilter();
  return instance;
}

void ObstacleFilter::AddObstacle(const std::shared_ptr<Obstacle> &obstacle_ptr) {
  obstacles_.emplace(obstacle_ptr->Id(), obstacle_ptr);
}

void ObstacleFilter::UpdateObstacles(const std::list<std::shared_ptr<Obstacle>> &obstacles) {
  obstacles_.clear();
  const double obstacle_filter_distance = 150.0;
  for (const auto &obstacle : obstacles) {

    if (obstacle->road_id() != VehicleState::Instance().road_id()) {
      continue;
    }
    double dist = std::hypot(obstacle->Center().x() - VehicleState::Instance().pose().position.x,
                             obstacle->Center().y() - VehicleState::Instance().pose().position.y);
    if (dist > obstacle_filter_distance) {
      continue;
    }
    obstacles_.emplace(obstacle->Id(), obstacle);
  }
}

}