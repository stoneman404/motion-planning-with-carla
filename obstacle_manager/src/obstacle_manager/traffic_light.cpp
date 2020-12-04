#include <tf/transform_datatypes.h>
#include "math/math_utils.hpp"
#include "obstacle_manager/traffic_light.hpp"

namespace planning {
TrafficLight::TrafficLight(const TrafficLight &other) {
  this->traffic_light_box_ = other.traffic_light_box_;
  this->traffic_light_status_ = other.traffic_light_status_;
  transform_ = other.transform_;
  this->id_ = other.id_;

}

void TrafficLight::UpdateTrafficLightStatus(const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
                                            const carla_waypoint_types::CarlaWaypoint &carla_waypoint) {
  traffic_light_status_ = traffic_light_status;
}

const common::Box2d &TrafficLight::GetBox2d() const { return traffic_light_box_; }
const uint32_t &TrafficLight::Id() const { return id_; }
//const uint32_t &TrafficLight::LaneId() const { return lane_id_; }
//const uint32_t &TrafficLight::SectionId() const { return section_id_; }
//const uint32_t &TrafficLight::RoadId() const { return road_id_; }
const geometry_msgs::Pose &TrafficLight::Transform() const { return transform_; }
TrafficLight::TrafficLight(const carla_msgs::CarlaTrafficLightInfo &traffic_light_info,
                           const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
                           const carla_waypoint_types::CarlaWaypoint &carla_waypoint)
    : id_(traffic_light_info.id),
      traffic_light_status_(traffic_light_status),
      transform_(traffic_light_info.transform) {

}
}