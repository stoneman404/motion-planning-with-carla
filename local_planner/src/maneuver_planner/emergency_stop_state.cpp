#include "obstacle_filter/obstacle_filter.hpp"
#include "traffic_lights/traffic_light_list.hpp"
#include "maneuver_planner/follow_lane_state.hpp"
#include "maneuver_planner/emergency_stop_state.hpp"
#include "maneuver_planner/stop_state.hpp"
#include "maneuver_planner/change_left_lane_state.hpp"
#include "maneuver_planner/change_right_lane_state.hpp"
namespace planning {

bool EmergencyStopState::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("Oops, something going wrong, we enter the **EmergencyStopState**");
  reference_line_ = PlanningContext::Instance().reference_lines().back(); //  reference line
}

bool EmergencyStopState::Execute(ManeuverPlanner *maneuver_planner) {
  if (maneuver_planner == nullptr) {
    return false;
  }
  // todo emergency stop trajectory motion_planner
  return false;
}

void EmergencyStopState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are exiting **EmergencyStopState**");
}

State &EmergencyStopState::Instance() {
  static EmergencyStopState instance;
  return instance;
}
std::string EmergencyStopState::Name() const { return "EmergencyStopState"; }

State *EmergencyStopState::NextState(ManeuverPlanner *maneuver_planner) const {
  if (maneuver_planner == nullptr) {
    return nullptr;
  }
  ManeuverGoal obstacle_maneuver;
  ManeuverGoal traffic_light_maneuver;
  this->ObstacleDecision(&obstacle_maneuver);
  this->TrafficLightDecision(reference_line_, &traffic_light_maneuver);
  auto combined_maneuver = CombineManeuver(traffic_light_maneuver, obstacle_maneuver);
  switch (combined_maneuver.decision_type) {
    case DecisionType::kFollowLane: return &(FollowLaneState::Instance());
    case DecisionType::kEmergencyStop: return &(EmergencyStopState::Instance());
    case DecisionType::kStopAtTrafficSign:
    case DecisionType::kStopAtDestination: return &(StopState::Instance());
    case DecisionType::kChangeRight: return &(ChangeRightLaneState::Instance());
    case DecisionType::kChangeLeft: return &(ChangeLeftLaneState::Instance());
    default:return nullptr;
  }
}

void EmergencyStopState::ObstacleDecision(ManeuverGoal *maneuver_goal) const {
  double forward_clear_distance, backward_clear_distance;
  int leading_vehicle_id, following_vehicle_id;
  SLPoint ego_sl;
  reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                          VehicleState::Instance().pose().position.y,
                          &ego_sl);
  this->GetLaneClearDistance(0,
                             reference_line_,
                             &forward_clear_distance,
                             &backward_clear_distance,
                             &leading_vehicle_id,
                             &following_vehicle_id);

  if (forward_clear_distance > PlanningConfig::Instance().lon_safety_buffer()
      && backward_clear_distance > PlanningConfig::Instance().lon_safety_buffer()) {
    // following lane
    maneuver_goal->decision_type = DecisionType::kFollowLane;
    maneuver_goal->has_stop_point = false;
    maneuver_goal->target_speed =
        leading_vehicle_id < 0 ?
        PlanningConfig::Instance().target_speed() : ObstacleFilter::Instance().Obstacles().at(
            leading_vehicle_id)->Speed();
    maneuver_goal->target_s = ego_sl.s +
        std::min(PlanningConfig::Instance().max_lookahead_distance(),
                 std::max(PlanningConfig::Instance().min_lookahead_distance(),
                          forward_clear_distance
                              - PlanningConfig::Instance().lon_safety_buffer()));
    maneuver_goal->lane_id = reference_line_->NearestWayPoint(maneuver_goal->target_s).lane_id;

  } else {
    // emergency stop
    maneuver_goal->decision_type = DecisionType::kEmergencyStop;
    maneuver_goal->has_stop_point = true;
    maneuver_goal->target_speed = 0.0;
    maneuver_goal->target_s = ego_sl.s + PlanningConfig::Instance().lon_safety_buffer();
    maneuver_goal->lane_id = reference_line_->NearestWayPoint(maneuver_goal->target_s).lane_id;
  }
}
}