#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_HPP_
#include <carla_msgs/CarlaTrafficLightInfo.h>
#include <carla_msgs/CarlaBoundingBox.h>
#include <carla_msgs/CarlaTrafficLightStatus.h>

namespace planning{
class TrafficLight{
 public:
  TrafficLight();
  ~TrafficLight();
  
};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_TRAFFIC_LIGHTS_TRAFFIC_LIGHT_HPP_
