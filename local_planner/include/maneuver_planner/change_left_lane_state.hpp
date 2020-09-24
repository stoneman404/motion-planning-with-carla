#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LEFT_LANE_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LEFT_LANE_STATE_HPP_
#include "state.hpp"
namespace planning {
class ChangeLeftLaneState : public State {
 public:
  bool Enter(ManeuverPlanner *maneuver_planner) override;
  bool Execute(ManeuverPlanner *maneuver_planner) override;
  void Exit(ManeuverPlanner *maneuver_planner) override;
  std::string Name() const override;
  static State &Instance();
  State *NextState(ManeuverPlanner *maneuver_planner) const override;
 private:
  ChangeLeftLaneState() = default;
  ChangeLeftLaneState(const ChangeLeftLaneState &other);
  ChangeLeftLaneState &operator=(const ChangeLeftLaneState &other);
};
}
#endif //
