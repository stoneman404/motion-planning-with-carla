#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_EMERGENCY_STOP_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_EMERGENCY_STOP_HPP_
#include "state.hpp"

namespace planning {
class EmergencyStop : public State {
 public:
  bool Enter(ManeuverPlanner *maneuver_planner) override;
  void Exit(ManeuverPlanner *maneuver_planner) override;
  ManeuverStatus Execute(ManeuverPlanner *maneuver_planner) override;
  static State &Instance();
  std::string Name() const override;
  State *NextState(ManeuverPlanner *maneuver_planner) const override;
 protected:
  void ObstacleDecision(ManeuverGoal *maneuver_goal) const override;
 private:
  EmergencyStop() = default;
  EmergencyStop(const EmergencyStop &other);
  EmergencyStop &operator=(const EmergencyStop &other);
 private:
  std::shared_ptr<ReferenceLine> reference_line_;
  int current_lane_id_{};
};
}
#endif
