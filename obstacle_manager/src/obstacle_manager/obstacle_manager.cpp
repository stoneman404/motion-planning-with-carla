#include "obstacle_manager/obstacle_manager.hpp"
namespace planning{
ObstacleManager &planning::ObstacleManager::Instance() {
  static ObstacleManager instance;
  return instance;
}

std::shared_ptr<Obstacle> ObstacleManager::GetObstacle(int id) const {
  if (obstacles_.empty() || obstacles_.find(id) == obstacles_.end()){
    return nullptr;

  } else {
    return obstacles_.at(id);
  }
}

void ObstacleManager::UpdateObstacles(const std::vector<std::shared_ptr<Obstacle>> &obstacles) {
  obstacles_.clear();
  const double obstacle_filter_distance = 150.0;
  for (const auto &obstacle : obstacles) {
    obstacles_.emplace(obstacle->Id(), obstacle);
  }
}
void ObstacleManager::AddTrafficLight(const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
                                      const carla_msgs::CarlaTrafficLightInfo &traffic_light_info,
                                      const carla_waypoint_types::CarlaWaypoint &carla_waypoint) {
  int traffic_light_id = traffic_light_info.id;
  if (traffic_lights_.find(traffic_light_id) != traffic_lights_.end()) {
    traffic_lights_[traffic_light_id]->UpdateTrafficLightStatus(traffic_light_status, carla_waypoint);
  } else {
    auto traffic_light = std::make_shared<TrafficLight>(traffic_light_info, traffic_light_status, carla_waypoint);
    traffic_lights_.emplace(traffic_light_id, traffic_light);
  }

}
const std::unordered_map<int, std::shared_ptr<TrafficLight>> &ObstacleManager::TrafficLights() const {
  return traffic_lights_;
}
const std::unordered_map<int, std::shared_ptr<Obstacle>> &ObstacleManager::Obstacles() const {return obstacles_;}
std::shared_ptr<TrafficLight> ObstacleManager::GetTrafficLight(int id) const {
  if (traffic_lights_.empty() || traffic_lights_.find(id) == traffic_lights_.end()) {
    return nullptr;
  } else {
    return traffic_lights_.at(id);
  }
}

}

