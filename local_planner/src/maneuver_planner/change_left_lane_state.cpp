#include "maneuver_planner/change_left_lane_state.hpp"
namespace planning {

bool ChangeLeftLaneState::Enter(ManeuverPlanner *maneuver_planner) {
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

bool ChangeLeftLaneState::Execute(ManeuverPlanner *maneuver_planner) {

  if (maneuver_planner == nullptr) {
    ROS_FATAL("[ChangeLeftLaneState::Execute] failed, maneuver_planner == nullptr");
    return false;
  }

  return false;
}
void ChangeLeftLaneState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are currently exit the **ChangeLeftLaneState**");
  PlanningContext::Instance().mutable_route_infos().pop_front();
  PlanningContext::Instance().mutable_reference_lines().pop_front();
}

State &ChangeLeftLaneState::Instance() {
  static ChangeLeftLaneState instance;
  return instance;
}

std::string ChangeLeftLaneState::Name() const { return "ChangeLeftLaneState"; }

State *ChangeLeftLaneState::NextState(ManeuverPlanner *maneuver_planner) const {
  int ego_lane_id = VehicleState::Instance().lane_id();
  int target_lane_id = maneuver_planner->maneuver_goal().lane_id;
  ManeuverGoal obstacle_maneuver;
  ManeuverGoal traffic_light_maneuver;
  this->ObstacleDecision(&obstacle_maneuver);
  this->TrafficLightDecision(&traffic_light_maneuver);
  if (ego_lane_id == target_lane_id) {
    return &(FollowLaneState::Instance());
  }
  return nullptr;
}
void ChangeLeftLaneState::TrafficLightDecision(ManeuverGoal *maneuver_goal) const {

}

void ChangeLeftLaneState::ObstacleDecision(ManeuverGoal *maneuver_goal) const {
  double leading_clear_distance;
  double following_clear_distance;
  int leading_vehicle_id;
  int following_vehicle_id;
  const double max_decel = PlanningConfig::Instance().max_acc();

  if (current_lane_id_ == target_lane_id_) {
    this->GetLaneClearDistance(0, target_reference_line_,
                               &leading_clear_distance,
                               &following_clear_distance,
                               &leading_vehicle_id,
                               &following_vehicle_id);

    maneuver_goal->decision_type = DecisionType::kFollowLane;
  } else {

  }

}

}