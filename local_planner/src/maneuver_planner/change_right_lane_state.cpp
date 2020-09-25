#include "maneuver_planner/state.hpp"
#include "maneuver_planner/change_right_lane_state.hpp"

namespace planning {

bool ChangeRightLaneState::Execute(ManeuverPlanner *maneuver_planner) {

  return false;
}

bool ChangeRightLaneState::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current enter the ChangeRightLane State");

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
  return nullptr;
}
void ChangeRightLaneState::TrafficLightDecision(ManeuverGoal *maneuver_goal) const {

}
void ChangeRightLaneState::ObstacleDecision(ManeuverGoal *maneuver_goal) const {

}

}