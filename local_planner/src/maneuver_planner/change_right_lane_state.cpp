#include "maneuver_planner/state.hpp"
#include "maneuver_planner/change_right_lane_state.hpp"

namespace planning {

bool ChangeRightLaneState::Execute(ManeuverPlanner *maneuver_planner) {

  if (maneuver_planner == nullptr) {
    return false;
  }
  // todo trajectory planner

  return false;
}

bool ChangeRightLaneState::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current enter the ChangeRightLane State");
  // todo: set offset for ego_pose
  const auto ego_pose = VehicleState::Instance().pose();
  const auto goal_pose = PlanningContext::Instance().global_goal_pose().pose;
  if (maneuver_planner->NeedReRoute()) {
    planning_srvs::RouteResponse route_response;
    bool result = maneuver_planner->ReRoute(ego_pose, goal_pose, route_response);
    if (!result) {
      ROS_FATAL("Failed to enter **ChangeLeftLaneState** state");
      return false;
    }
    auto &route_infos = PlanningContext::Instance().mutable_route_infos();
    route_infos.push_back(route_response);
    std::list<std::shared_ptr<ReferenceLine>> reference_lines;
    bool reference_result = maneuver_planner->UpdateReferenceLine(&reference_lines);
    if (!reference_result) {
      ROS_FATAL("Failed to enter **ChangeLeftLaneState**, because UpdateReferenceLine failed");
      return false;
    }
    auto &reference_line_list = PlanningContext::Instance().mutable_reference_lines();
    reference_line_list = reference_lines;
  }
  target_lane_id_ = maneuver_planner->maneuver_goal().lane_id;
  current_lane_id_ = VehicleState::Instance().lane_id();
  ROS_ASSERT(PlanningContext::Instance().mutable_reference_lines().size() == 2);
  current_reference_line_ = PlanningContext::Instance().reference_lines().front();
  target_reference_line_ = PlanningContext::Instance().reference_lines().back();
}

void ChangeRightLaneState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current exit the ChangeRightLane State");
}

State &ChangeRightLaneState::Instance() {
  static ChangeRightLaneState instance;
  return instance;
}

std::string ChangeRightLaneState::Name() const { return "ChangeRightLaneState"; }

State *ChangeRightLaneState::NextState(ManeuverPlanner *maneuver_planner) const {
  if (maneuver_planner == nullptr) {
    return nullptr;
  }

}

void ChangeRightLaneState::TrafficLightDecision(ManeuverGoal *maneuver_goal) const {
  return;
}
void ChangeRightLaneState::ObstacleDecision(ManeuverGoal *maneuver_goal) const {

}

}