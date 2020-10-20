#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LEFT_LANE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LEFT_LANE_HPP_
#include "state.hpp"
#include "follow_lane.hpp"
#include "stop.hpp"
#include "emergency_stop.hpp"
namespace planning {
class ChangeLeftLane : public State {
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
  void ObstacleDecision(ManeuverGoal *maneuver_goal) const override;
 private:
  ChangeLeftLane() = default;
  ChangeLeftLane(const ChangeLeftLane &other);
  ChangeLeftLane &operator=(const ChangeLeftLane &other);

 private:
  int after_lane_id_{};
  int before_lane_id_{};
  std::shared_ptr<ReferenceLine> before_reference_line_{};
  std::shared_ptr<ReferenceLine> after_reference_line_{};
};
}
#endif //
