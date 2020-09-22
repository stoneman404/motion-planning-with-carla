#include "maneuver_planner/overtake_state.hpp"
namespace planning {

bool OvertakeState::Enter(ManeuverPlanner *maneuver_planner) {

}
bool OvertakeState::Execute(ManeuverPlanner *maneuver_planner) {
  return false;
}
void OvertakeState::Exit(ManeuverPlanner *maneuver_planner) {

}
State &OvertakeState::Instance() {
  static OvertakeState instance;
  return instance;
}
std::string OvertakeState::Name() const { return "OvertakeState"; }
State *OvertakeState::NextState(ManeuverPlanner *maneuver_planner) const {
  return nullptr;
}

}