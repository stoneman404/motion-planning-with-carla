#include <tf/transform_datatypes.h>
#include "math_utils.hpp"
#include "traffic_lights/traffic_light.hpp"

namespace planning {
TrafficLight::TrafficLight(const TrafficLight &other) {
  this->traffic_light_box_ = other.traffic_light_box_;
  trigger_volume_ = other.trigger_volume_;
  this->traffic_light_status_ = other.traffic_light_status_;
  transform_ = other.transform_;
  this->id_ = other.id_;
  this->lane_id_ = other.lane_id_;
  this->section_id_ = other.section_id_;
  this->road_id_ = other.road_id_;
}

void TrafficLight::UpdateTrafficLightStatus(const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
                                            const carla_waypoint_types::CarlaWaypoint &carla_waypoint) {
  traffic_light_status_ = traffic_light_status;
  lane_id_ = carla_waypoint.lane_id;
  section_id_ = carla_waypoint.section_id;
  road_id_ = carla_waypoint.road_id;
}

const Box2d &TrafficLight::GetBox2d() const { return traffic_light_box_; }
const uint32_t &TrafficLight::Id() const { return id_; }
const uint32_t &TrafficLight::LaneId() const { return lane_id_; }
const uint32_t &TrafficLight::SectionId() const { return section_id_; }
const uint32_t &TrafficLight::RoadId() const { return road_id_; }
const geometry_msgs::Pose &TrafficLight::Transform() const { return transform_; }
TrafficLight::TrafficLight(const carla_msgs::CarlaTrafficLightInfo &traffic_light_info,
                           const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
                           const carla_waypoint_types::CarlaWaypoint &carla_waypoint)
    : id_(traffic_light_info.id),
      lane_id_(carla_waypoint.lane_id),
      section_id_(carla_waypoint.section_id),
      road_id_(carla_waypoint.road_id),
      trigger_volume_(traffic_light_info.trigger_volume),
      traffic_light_status_(traffic_light_status),
      transform_(traffic_light_info.transform) {
  const geometry_msgs::Point light_location = transform_.position;
  const double light_yaw = tf::getYaw(transform_.orientation);
  Eigen::Vector3d center_point;
  center_point << trigger_volume_.center.x, trigger_volume_.center.y, trigger_volume_.center.z;
  const auto traffic_light_center = MathUtils::Transform(transform_, center_point);
  Eigen::Vector2d center_xy;
  center_xy << traffic_light_center.x(), traffic_light_center.y();
  traffic_light_box_ = Box2d(center_xy, light_yaw, trigger_volume_.size.x, trigger_volume_.size.y);
}
}