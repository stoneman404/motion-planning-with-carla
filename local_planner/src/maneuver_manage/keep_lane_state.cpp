#include <maneuver_manage/emergency_stop_state.hpp>
#include <obstacle_filter/obstacle_filter.hpp>
#include <reference_line/reference_line.hpp>
#include "maneuver_manage/keep_lane_state.hpp"
#include "planning_config.hpp"
#include "planning_context.hpp"
namespace planning {

bool KeepLaneState::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current switching to KeepLaneState");
  const auto ego_pose = VehicleState::Instance().pose();
  const auto lane_id = VehicleState::Instance().lane_id();
  const auto goal_pose = PlanningContext::Instance().global_goal_pose().pose;
  planning_srvs::RouteResponse route_response;
  bool result = maneuver_planner->ReRoute(ego_pose, goal_pose, route_response);
  if (!result){
    ROS_FATAL("Failed to enter KeepLaneState state");
    return false;
  }
  auto route_infos = PlanningContext::Instance().mutable_route_infos();
  if (!route_infos.empty()){
    route_infos.clear();
  }
  route_infos.push_back(route_response);
}

bool KeepLaneState::Execute(ManeuverPlanner *maneuver_planner) {

  if (maneuver_planner == nullptr) {
    ROS_ERROR("the ManeuverPlanner is nullptr");
    return false;
  }
  std::list<std::shared_ptr<ReferenceLine>> reference_lines;
  bool reference_result = maneuver_planner->UpdateReferenceLine(&reference_lines);
  if (!reference_result){
    return false;
  }

  if (VehicleState::Instance().lane_id() == -1
      || VehicleState::Instance().section_id() == -1
      || VehicleState::Instance().road_id() == -1) {
    ROS_ERROR("the ego vehicle is currently out of lane");
    maneuver_planner->SetState(EmergencyStopState::Instance());
    return true;
  }

  const double safety_buffer = PlanningConfig::Instance().safety_buffer();
  const double max_lookahead_time = PlanningConfig::Instance().max_lookahead_time();
  auto obstacles = ObstacleFilter::Instance().Obstacles();

  for (const auto &obstacle : obstacles) {
    // ignore the vehicle and pedestrians in other lane or road
    if (obstacle->road_id() != VehicleState::Instance().road_id()
        || obstacle->section_id() != VehicleState::Instance().section_id()
        || obstacle->lane_id() != VehicleState::Instance().lane_id()) {
      continue;
    }
    const double obstacle_speed = obstacle->Speed();

  }

}
void KeepLaneState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are currently Exiting the KeepLaneState");
}
State &KeepLaneState::Instance() {
  static KeepLaneState instance;
  return instance;
}

double KeepLaneState::TransitionFunction() {

}

}
