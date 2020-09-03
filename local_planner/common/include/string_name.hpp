#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_STRING_NAME_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_STRING_NAME_HPP_
#include <string>
namespace planning{
namespace topic{
const std::string kEgoVehicleStatusName = "/carla/ego_vehicle/vehicle_stauts";
const std::string kTrafficLigthsName = "/carla/traffic_lights";
const std::string kCarlaActorListName = "/carla/actor_list";
const std::string kObjectsName = "/carla/objects";  // contains the ego vehicle
const std::string kPublishedTrajectoryName = "/published_trajectory";
const std::string kEgoVehicleInfoName = "/carla/ego_vehicle/vehicle_info";
const std::string kEgoVehicleOdometryName = "/carla/ego_vehicle/odometry";
const std::string kVisualizedTrajectoryName = "/visualized_trajectory";
const std::string kInitialPoseName = "/initialpose";
const std::string kGoalPoseName = "/move_base_simple/goal";
const std::string kTrafficLightsInfoName = "/carla/"
;
}

namespace service{
const std::string kRouteServiceName = "/carla/ego_vehicle/get_route";
const std::string kGetActorWaypointServiceName = "/carla_waypoint_publisher/ego_vehicle/get_actor_waypoint";
const std::string kGetEgoWaypontServiceName = "/carla_waypoint_publisher/ego_vehicle/get_waypoint";

}
}

#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_STRING_NAME_HPP_
