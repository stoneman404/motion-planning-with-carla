#include "maneuver_manage/change_lane_right_state.hpp"
namespace planning{

void ChangeLaneRight::Enter(ManeuverPlanner* manuever_planner) {

}
bool ChangeLaneRight::Execute(ManeuverPlanner* manuever_planner) {
  return false;
}
void ChangeLaneRight::Exit(ManeuverPlanner* manuever_planner) {

}
State &ChangeLaneRight::Instance() {
  static ChangeLaneRight instance;
  return instance;
}
}