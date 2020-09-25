#include "maneuver_planner/stop_state.hpp"
namespace planning {

bool StopState::Enter(ManeuverPlanner *maneuver_planner) {

}

void StopState::Exit(ManeuverPlanner *maneuver_planner) {

}

bool StopState::Execute(ManeuverPlanner *maneuver_planner) {

  return false;
}

State &StopState::Instance() {
  static StopState instance;
  return instance;
}

std::string StopState::Name() const { return "StopState"; }

State *StopState::NextState(ManeuverPlanner *maneuver_planner) const {

  return nullptr;
}

void StopState::ObstacleDecision(ManeuverGoal *maneuver_goal) const {
}

void StopState::TrafficLightDecision(ManeuverGoal *maneuver_goal) const {

}

}