#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LEFT_LANE_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LEFT_LANE_STATE_HPP_
#include "state.hpp"
#include "follow_lane_state.hpp"
#include "stop_state.hpp"
#include "emergency_stop_state.hpp"
namespace planning {
class ChangeLeftLaneState : public State {
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
  ChangeLeftLaneState() = default;
  ChangeLeftLaneState(const ChangeLeftLaneState &other);
  ChangeLeftLaneState &operator=(const ChangeLeftLaneState &other);
 private:
  int target_lane_id_{};
  int current_lane_id_{};
  std::shared_ptr<ReferenceLine> current_reference_line_{};
  std::shared_ptr<ReferenceLine> target_reference_line_{};
};
}
#endif //
