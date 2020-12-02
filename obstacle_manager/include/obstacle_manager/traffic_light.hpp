#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_HPP_
#include <carla_msgs/CarlaTrafficLightInfo.h>
#include <carla_msgs/CarlaBoundingBox.h>
#include <carla_msgs/CarlaTrafficLightStatus.h>
#include <geometry_msgs/Pose.h>
#include <carla_waypoint_types/CarlaWaypoint.h>
#include "polygon/box2d.hpp"

namespace planning {
class TrafficLight {

 public:
  TrafficLight() = default;
  TrafficLight(const carla_msgs::CarlaTrafficLightInfo &traffic_light_info,
               const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
               const carla_waypoint_types::CarlaWaypoint &carla_waypoint);
  ~TrafficLight() = default;
  TrafficLight(const TrafficLight &other);
  void UpdateTrafficLightStatus(const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
                                const carla_waypoint_types::CarlaWaypoint &carla_waypoint);

  const carla_msgs::CarlaTrafficLightStatus &TrafficLightStatus() const {
    return traffic_light_status_;
  }
  const common::Box2d& GetBox2d() const;;
  const uint32_t &Id() const;
//  const uint32_t &LaneId() const;
//  const uint32_t &SectionId() const;
//  const uint32_t &RoadId() const;
  const geometry_msgs::Pose &Transform() const;

 private:
  uint32_t id_{};
  uint32_t lane_id_{};
  uint32_t section_id_{};
  uint32_t road_id_{};
  common::Box2d traffic_light_box_;
  carla_msgs::CarlaBoundingBox trigger_volume_;
  carla_msgs::CarlaTrafficLightStatus traffic_light_status_;
  geometry_msgs::Pose transform_;

};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_HPP_
