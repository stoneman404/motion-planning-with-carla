#include "maneuver_planner/change_left_lane_state.hpp"
namespace planning {

bool ChangeLeftLaneState::Enter(ManeuverPlanner *maneuver_planner) {

}
bool ChangeLeftLaneState::Execute(ManeuverPlanner *maneuver_planner) {
  return false;
}
void ChangeLeftLaneState::Exit(ManeuverPlanner *maneuver_planner) {

}
State &ChangeLeftLaneState::Instance() {
  static ChangeLeftLaneState instance;
  return instance;
}
std::string ChangeLeftLaneState::Name() const { return "ChangeLeftLaneState"; }
State *ChangeLeftLaneState::NextState(ManeuverPlanner *maneuver_planner) const {
  return nullptr;
}

}