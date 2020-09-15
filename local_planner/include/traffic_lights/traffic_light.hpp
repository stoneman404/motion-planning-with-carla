#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_HPP_
#include <carla_msgs/CarlaTrafficLightInfo.h>
#include <carla_msgs/CarlaBoundingBox.h>
#include <carla_msgs/CarlaTrafficLightStatus.h>
#include <geometry_msgs/Pose.h>
#include <carla_waypoint_types/CarlaWaypoint.h>

namespace planning {
class TrafficLight {

 public:
  TrafficLight() = default;
  TrafficLight(const carla_msgs::CarlaTrafficLightInfo &traffic_light_info,
               const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
               const carla_waypoint_types::CarlaWaypoint& carla_waypoint);
  ~TrafficLight() = default;
  void UpdateTrafficLightStatus(const carla_msgs::CarlaTrafficLightStatus& traffic_light_status,
                                const carla_waypoint_types::CarlaWaypoint& carla_waypoint);

  const carla_msgs::CarlaTrafficLightStatus &TrafficLightStatus() const;
  const carla_msgs::CarlaBoundingBox& TrafficLightBoundingBox() const;
  const int& Id() const;
  const int& LaneId() const;
  const int& SectionId() const;
  const int& RoadId() const;

 private:
  carla_msgs::CarlaBoundingBox trigger_volume_;
  carla_msgs::CarlaTrafficLightStatus traffic_light_status_;
  geometry_msgs::Pose transform_;
  int id_{-1};
  int lane_id_{-1};
  int section_id_{-1};
  int road_id_{-1};
};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_HPP_
