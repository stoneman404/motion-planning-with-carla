#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_LIST_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_LIST_HPP_
#include <unordered_map>
#include <ros/ros.h>
#include "traffic_light.hpp"
namespace planning {
class TrafficLightList {
 public:
  static TrafficLightList &Instance();
  const std::unordered_map<int, std::shared_ptr<TrafficLight>> &TrafficLights() const {
    ROS_INFO("[TrafficLights] size: %zu", traffic_lights_.size());
    return traffic_lights_;
  }
  void AddTrafficLight(const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
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

 private:
  std::unordered_map<int, std::shared_ptr<TrafficLight>> traffic_lights_{};

 private:
  TrafficLightList() = default;
  ~TrafficLightList() = default;
};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_LIST_HPP_
