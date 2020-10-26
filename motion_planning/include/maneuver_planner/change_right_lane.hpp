
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_RIGHT_LANE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_RIGHT_LANE_HPP_
#include "state.hpp"
namespace planning {
class ChangeRightLane : public State {
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
//  ManeuverStatus Execute(ManeuverPlanner *maneuver_planner) override;

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
  State *Transition(ManeuverPlanner *maneuver_planner) override;

 protected:
  /**
   *
   * @param maneuver_goal
   */
  void ObstacleDecision(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                        ManeuverGoal *maneuver_goal) const override;

 private:
  ChangeRightLane() = default;
  ChangeRightLane(const ChangeRightLane &other);
  ChangeRightLane &operator=(const ChangeRightLane &other);
 private:
  int after_lane_id_{};
  int before_lane_id_{};
  std::shared_ptr<ReferenceLine> before_reference_line_{};
  std::shared_ptr<ReferenceLine> after_reference_line_{};
};

}
#endif //
