#include "maneuver_manage/emergency_stop_state.hpp"
namespace planning {

bool EmergencyStopState::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("Oops, something going wrong, we Enter the EmergecyStopState");
}

bool EmergencyStopState::Execute(ManeuverPlanner *maneuver_planner) {
  return false;
}

void EmergencyStopState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We have recovered from EmergencyStopState");
}

State &EmergencyStopState::Instance() {
  static EmergencyStopState instance;
  return instance;
}
std::string EmergencyStopState::Name() const {return "EmergencyStopState";}

State *EmergencyStopState::NextState(ManeuverPlanner *maneuver_planner) const {
  return nullptr;
}

}