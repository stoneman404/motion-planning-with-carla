#include "maneuver_planner/emergency_stop_state.hpp"
#include "maneuver_planner/follow_lane_state.hpp"
#include "maneuver_planner/stop_state.hpp"
#include "obstacle_filter/obstacle_filter.hpp"

namespace planning {

bool StopState::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are currently switching to **StopState**");
  ROS_ASSERT(maneuver_planner->multable_maneuver_goal().maneuver_infos.size() == 1);
  reference_line_ = maneuver_planner->multable_maneuver_goal().maneuver_infos.front().ptr_ref_line;
  current_lane_id_ = maneuver_planner->multable_maneuver_goal().maneuver_infos.front().lane_id;
}

void StopState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are currently exiting **StopState**");
}

bool StopState::Execute(ManeuverPlanner *maneuver_planner) {
  if (nullptr == maneuver_planner) {
    return false;
  }
  // todo stop trajectory motion_planner
  return false;
}

State &StopState::Instance() {
  static StopState instance;
  return instance;
}

std::string StopState::Name() const { return "StopState"; }

State *StopState::NextState(ManeuverPlanner *maneuver_planner) const {
  if (maneuver_planner == nullptr) {
    return nullptr;
  }
  ManeuverGoal obstacle_maneuver;
  ManeuverGoal traffic_light_maneuver;
  this->ObstacleDecision(&obstacle_maneuver);
  this->TrafficLightDecision(reference_line_, &traffic_light_maneuver);
  auto combined_maneuver = CombineManeuver(traffic_light_maneuver, obstacle_maneuver);
  switch (combined_maneuver.decision_type) {
    case DecisionType::kStopAtTrafficSign:
    case DecisionType::kStopAtDestination:return &(StopState::Instance());
    case DecisionType::kEmergencyStop:return &(EmergencyStopState::Instance());
    case DecisionType::kFollowLane:return &(FollowLaneState::Instance());
    case DecisionType::kChangeLeft:return &(ChangeLeftLaneState::Instance());
    case DecisionType::kChangeRight:return &(ChangeRightLaneState::Instance());
    default:return nullptr;
  }
}

void StopState::ObstacleDecision(ManeuverGoal *maneuver_goal) const {
  double following_clear_distance, leading_clear_distance;
  int leading_vehicle_id, following_vehicle_state;
  const auto obstacles = ObstacleFilter::Instance().Obstacles();
  SLPoint ego_sl;
  reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                          VehicleState::Instance().pose().position.y,
                          &ego_sl);
  maneuver_goal->maneuver_infos.resize(1);
  this->GetLaneClearDistance(0,
                             reference_line_,
                             &leading_clear_distance,
                             &following_clear_distance,
                             &leading_vehicle_id,
                             &following_vehicle_state);
  if (leading_clear_distance < PlanningConfig::Instance().lon_safety_buffer() &&
      following_clear_distance < PlanningConfig::Instance().lon_safety_buffer()) {
    maneuver_goal->decision_type = DecisionType::kEmergencyStop;
    maneuver_goal->maneuver_infos.front().has_stop_point = true;
    maneuver_goal->maneuver_infos.front().maneuver_target.target_s =
        std::min(ego_sl.s + std::max(leading_clear_distance - PlanningConfig::Instance().lon_safety_buffer(),
                                     PlanningConfig::Instance().min_lookahead_distance()),
                 PlanningConfig::Instance().max_lookahead_distance());
    maneuver_goal->maneuver_infos.front().lane_id = current_lane_id_;
    maneuver_goal->maneuver_infos.front().ptr_ref_line = reference_line_;
  } else {
    if (ego_sl.s + leading_clear_distance > reference_line_->Length()) {
      maneuver_goal->maneuver_infos.front().has_stop_point = true;
//      maneuver_goal->target_speed = 0.0;
      maneuver_goal->decision_type = DecisionType::kStopAtDestination;
      maneuver_goal->maneuver_infos.front().maneuver_target.target_s =
          ego_sl.s + std::min(PlanningConfig::Instance().max_lookahead_distance(),
                              std::max(leading_clear_distance
                                           - PlanningConfig::Instance().lon_safety_buffer(),
                                       PlanningConfig::Instance().min_lookahead_distance()));
      maneuver_goal->maneuver_infos.front().lane_id = current_lane_id_;
    } else {
      maneuver_goal->maneuver_infos.front().has_stop_point = false;
      maneuver_goal->maneuver_infos.front().maneuver_target.target_speed =
          leading_vehicle_id < 0 ? PlanningConfig::Instance().target_speed() :
          ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed();
      maneuver_goal->maneuver_infos.front().ptr_ref_line = reference_line_;
      maneuver_goal->decision_type = DecisionType::kFollowLane;
      maneuver_goal->maneuver_infos.front().lane_id = current_lane_id_;
    }
  }
}
}