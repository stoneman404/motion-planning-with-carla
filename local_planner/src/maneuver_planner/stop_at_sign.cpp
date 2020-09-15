#include "maneuver_planner/stop_at_sign.hpp"
namespace planning {


bool StopAtSignState::Enter(ManeuverPlanner *manuever_planner) {

}

void StopAtSignState::Exit(ManeuverPlanner *manuvever_planner) {

}

bool StopAtSignState::Execute(ManeuverPlanner *manuever_planner) {

  return false;
}

State &StopAtSignState::Instance() {
  static StopAtSignState instance;
  return instance;
}


std::string StopAtSignState::Name() const { return "StopAtSignState"; }


State *StopAtSignState::NextState(ManeuverPlanner *maneuver_planner) const {
  return nullptr;
}

std::vector<StateName> StopAtSignState::GetPosibileNextStates() const {
  return std::vector<StateName>();
}

}