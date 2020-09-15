#include "traffic_lights/traffic_light.hpp"

namespace planning {

TrafficLight::TrafficLight(
    const carla_msgs::CarlaTrafficLightInfo &traffic_light_info,
    const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
    const carla_waypoint_types::CarlaWaypoint &carla_waypoint)
    : trigger_volume_(traffic_light_info.trigger_volume),
      traffic_light_status_(traffic_light_status),
      transform_(traffic_light_info.transform),
      waypoint_(carla_waypoint),
      id_(traffic_light_info.id),
      lane_id_(carla_waypoint.lane_id),
      section_id_(carla_waypoint.section_id),
      road_id_(carla_waypoint.road_id) {}

const carla_msgs::CarlaTrafficLightStatus &TrafficLight::TrafficLightStatus() const {
  return traffic_light_status_;
}

const carla_msgs::CarlaBoundingBox &TrafficLight::TrafficLightBoundingBox() const {
  return trigger_volume_;
}

const int &TrafficLight::Id() const { return id_; }
const int &TrafficLight::LaneId() const { return lane_id_; }
const int &TrafficLight::RoadId() const { return road_id_; }
const int &TrafficLight::SectionId() const { return section_id_; }

void TrafficLight::UpdateTrafficLightStatus(const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
                                            const carla_waypoint_types::CarlaWaypoint &carla_waypoint) {
  traffic_light_status_ = traffic_light_status;
  lane_id_ = carla_waypoint.lane_id;
  section_id_ = carla_waypoint.section_id;
  road_id_ = carla_waypoint.road_id;
  waypoint_ = carla_waypoint;
}

const geometry_msgs::Pose &TrafficLight::Transform() const {return transform_;}

const carla_waypoint_types::CarlaWaypoint &TrafficLight::WayPoint() const {return waypoint_;}

}