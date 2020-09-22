#include "maneuver_planner/state.hpp"
#include "maneuver_planner/tailgating_state.hpp"

namespace planning {

bool TailgatingState::Execute(ManeuverPlanner *maneuver_planner) {

  return false;
}

bool TailgatingState::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current enter the ChangeLaneLeft State");

}

void TailgatingState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current exit the ChangeLaneLeft State");

}

State &TailgatingState::Instance() {
  static TailgatingState instance;
  return instance;
}

std::string TailgatingState::Name() const { return "TailgatingState"; }

State *TailgatingState::NextState(ManeuverPlanner *maneuver_planner) const {
  return nullptr;
}


}