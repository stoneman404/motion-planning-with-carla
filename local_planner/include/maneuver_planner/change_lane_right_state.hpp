#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LANE_RIGHT_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LANE_RIGHT_STATE_HPP_
#include "state.hpp"
namespace planning {
class ChangeLaneRightState : public State {
 public:
  bool Enter(ManeuverPlanner *manuever_planner) override;
  bool Execute(ManeuverPlanner *manuever_planner) override;
  void Exit(ManeuverPlanner *manuever_planner) override;
  std::string Name() const override;
  static State &Instance();
  State *NextState(ManeuverPlanner *maneuver_planner) const override;
  std::vector<StateName> GetPosibileNextStates() const override;
 private:
  ChangeLaneRightState() = default;
  ChangeLaneRightState(const ChangeLaneRightState &other);
  ChangeLaneRightState &operator=(const ChangeLaneRightState &other);
};
}
#endif //
