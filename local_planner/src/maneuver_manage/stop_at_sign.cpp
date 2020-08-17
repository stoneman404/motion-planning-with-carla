#include "maneuver_manage/stop_at_sign.hpp"
namespace planning {

void StopAtSign::Enter(ManeuverPlanner *manuever_planner) {

}
void StopAtSign::Exit(ManeuverPlanner *manuvever_planner) {

}
bool StopAtSign::Execute(ManeuverPlanner *manuever_planner) {

  return false;
}
State &StopAtSign::Instance() {
  static StopAtSign instance;
  return instance;
}
}