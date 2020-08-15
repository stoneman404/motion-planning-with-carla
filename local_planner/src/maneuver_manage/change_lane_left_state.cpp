#include "maneuver_manage/state.hpp"
#include "maneuver_manage/change_lane_left_state.hpp"
namespace planning{

bool ChangeLaneLeft::Execute(ManeuverPlanner* manuever_planner) {

  return false;
}

void ChangeLaneLeft::Enter(ManeuverPlanner* manuever_planner) {
  ROS_INFO("We are current enter the ChangeLaneLeft State");

}
void ChangeLaneLeft::Exit(ManeuverPlanner* manuever_planner) {
  ROS_INFO("We are current exit the ChangeLaneLeft State");

}

State &ChangeLaneLeft::Instance() {
  static ChangeLaneLeft instance;
  return instance;
}
}