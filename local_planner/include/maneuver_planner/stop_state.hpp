#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_STOP_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_STOP_STATE_HPP_
#include "state.hpp"

namespace planning {
class StopState : public State {
 public:
  bool Enter(ManeuverPlanner *maneuver_planner) override;
  bool Execute(ManeuverPlanner *maneuver_planner) override;
  void Exit(ManeuverPlanner *maneuver_planner) override;
  std::string Name() const override;
  static State &Instance();
  State *NextState(ManeuverPlanner *maneuver_planner) const override;
 protected:
  void ObstacleDecision(ManeuverGoal *maneuver_goal) const override;

 private:
  StopState() = default;
  StopState(const StopState &other);
  StopState &operator=(const StopState &other);
 private:
  std::shared_ptr<ReferenceLine> reference_line_;

};
}
#endif //
