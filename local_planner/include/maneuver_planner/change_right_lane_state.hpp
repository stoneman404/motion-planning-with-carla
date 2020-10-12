
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_RIGHT_LANE_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_RIGHT_LANE_STATE_HPP_
#include "state.hpp"
namespace planning {
class ChangeRightLaneState : public State {
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
  ChangeRightLaneState() = default;
  ChangeRightLaneState(const ChangeRightLaneState &other);
  ChangeRightLaneState &operator=(const ChangeRightLaneState &other);
 private:
  int target_lane_id_{};
  int current_lane_id_{};
  std::shared_ptr<ReferenceLine> current_reference_line_{};
  std::shared_ptr<ReferenceLine> target_reference_line_{};
};

}
#endif //
