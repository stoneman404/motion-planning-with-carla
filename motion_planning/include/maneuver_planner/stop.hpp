#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_STOP_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_STOP_HPP_
#include "state.hpp"

namespace planning {
class Stop : public State {
 public:
  bool Enter(ManeuverPlanner *maneuver_planner) override;
//  ManeuverStatus Execute(ManeuverPlanner *maneuver_planner) override;
  void Exit(ManeuverPlanner *maneuver_planner) override;
  std::string Name() const override;
  static State &Instance();
  State *Transition(ManeuverPlanner *maneuver_planner) override;
 protected:
  void ObstacleDecision(ManeuverGoal *maneuver_goal) const override;

 private:
  Stop() = default;
  Stop(const Stop &other);
  Stop &operator=(const Stop &other);
 private:
  std::shared_ptr<ReferenceLine> reference_line_;
  int current_lane_id_{};

};
}
#endif //
