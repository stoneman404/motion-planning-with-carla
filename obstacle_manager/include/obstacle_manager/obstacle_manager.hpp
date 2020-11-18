
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_OBSTACLE_MANAGER_INCLUDE_OBSTACLE_MANAGER_OBSTACLE_MANAGER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_OBSTACLE_MANAGER_INCLUDE_OBSTACLE_MANAGER_OBSTACLE_MANAGER_HPP_
#include <unordered_map>
#include <obstacle_manager/obstacle.hpp>
#include <obstacle_manager/traffic_light.hpp>

namespace planning {

class ObstacleManager {
 public:
  static ObstacleManager &Instance();

  void UpdateObstacles(const std::vector<std::shared_ptr<Obstacle>> &obstacles);

  void AddTrafficLight(const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
                       const carla_msgs::CarlaTrafficLightInfo &traffic_light_info,
                       const carla_waypoint_types::CarlaWaypoint &carla_waypoint);

  std::shared_ptr<Obstacle> GetObstacle(int id) const;

  std::shared_ptr<TrafficLight> GetTrafficLight(int id) const;

  const std::unordered_map<int, std::shared_ptr<TrafficLight>> &TrafficLights() const;

  const std::unordered_map<int, std::shared_ptr<Obstacle>> &Obstacles() const;

 private:
  std::unordered_map<int, std::shared_ptr<Obstacle>> obstacles_;
  std::unordered_map<int, std::shared_ptr<TrafficLight>> traffic_lights_;
 private:
  ObstacleManager() = default;
  ~ObstacleManager() = default;

};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_OBSTACLE_MANAGER_INCLUDE_OBSTACLE_MANAGER_OBSTACLE_MANAGER_HPP_
