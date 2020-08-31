#include "maneuver_manage/change_lane_right_state.hpp"
namespace planning {

bool ChangeLaneRightState::Enter(ManeuverPlanner *manuever_planner) {

}
bool ChangeLaneRightState::Execute(ManeuverPlanner *manuever_planner) {
  return false;
}
void ChangeLaneRightState::Exit(ManeuverPlanner *manuever_planner) {

}
State &ChangeLaneRightState::Instance() {
  static ChangeLaneRightState instance;
  return instance;
}
}