#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LEFT_LANE_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LEFT_LANE_STATE_HPP_
#include "state.hpp"
#include "follow_lane_state.hpp"
#include "stop_state.hpp"
#include "emergency_stop_state.hpp"
namespace planning {
class ChangeLeftLaneState : public State {
 public:
  /**
   *
   * @param maneuver_planner
   * @return
   */
  bool Enter(ManeuverPlanner *maneuver_planner) override;

  /**
   *
   * @param maneuver_planner
   * @return
   */
  bool Execute(ManeuverPlanner *maneuver_planner) override;

  /**
   *
   * @param maneuver_planner
   */
  void Exit(ManeuverPlanner *maneuver_planner) override;

  /**
   *
   * @return
   */
  std::string Name() const override;

  /**
   *
   * @return
   */
  static State &Instance();

  /**
   *
   * @param maneuver_planner
   * @return
   */
  State *NextState(ManeuverPlanner *maneuver_planner) const override;
 protected:

  /**
   *
   * @param maneuver_goal
   */
  void ObstacleDecision(ManeuverGoal *maneuver_goal) const override;
 private:
  ChangeLeftLaneState() = default;
  ChangeLeftLaneState(const ChangeLeftLaneState &other);
  ChangeLeftLaneState &operator=(const ChangeLeftLaneState &other);

 private:
  int after_lane_id_{};
  int before_lane_id_{};
  std::shared_ptr<ReferenceLine> before_reference_line_{};
  std::shared_ptr<ReferenceLine> after_reference_line_{};
};
}
#endif //
