#include "traffic_lights/traffic_light_list.hpp"
namespace planning {

TrafficLightList &TrafficLightList::Instance() {
  static TrafficLightList instance;
  return instance;
}
void TrafficLightList::AddTrafficLight(const TrafficLight &traffic_light) {
  if (traffic_lights_.find(traffic_light.Id()) != traffic_lights_.end()) {
    traffic_lights_[traffic_light.Id()] = traffic_light;
  } else {
    traffic_lights_.emplace(traffic_light.Id(), traffic_light);
  }
}
void TrafficLightList::AddTrafficLight(const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
                                       const carla_msgs::CarlaTrafficLightInfo &traffic_light_info,
                                       const carla_waypoint_types::CarlaWaypoint &carla_waypoint) {
  int traffic_light_id = traffic_light_info.id;
  if (traffic_lights_.find(traffic_light_id) != traffic_lights_.end()) {
    traffic_lights_[traffic_light_id].UpdateTrafficLightStatus(traffic_light_status, carla_waypoint);
  } else {
    auto traffic_light = TrafficLight(traffic_light_info, traffic_light_status, carla_waypoint);
    traffic_lights_.emplace(traffic_light.Id(), traffic_light);
  }
}

}