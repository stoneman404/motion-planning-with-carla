#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_FOLLOW_LANE_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_FOLLOW_LANE_STATE_HPP_
#include "state.hpp"
#include <vector>
#include "planning_context.hpp"
#include "overtake_state.hpp"
#include "tailgating_state.hpp"
#include "stop_state.hpp"
#include "emergency_stop_state.hpp"

namespace planning {

class FollowLaneState : public State {

 public:
  bool Enter(ManeuverPlanner *maneuver_planner) override;
  bool Execute(ManeuverPlanner *maneuver_planner) override;
  void Exit(ManeuverPlanner *maneuver_planner) override;
  std::string Name() const override;
  static State &Instance();
  State *NextState(ManeuverPlanner *maneuver_planner) const override;

 private:
  void GetLaneClearDistance(int lane_offset,
                            double *const forward_clear_distance,
                            double *const backward_clear_distance,
                            int* const forward_obstacle_id, int* const backward_obstacle_id) const;
  DecisionType TrafficLightsDecision(ManeuverGoal* maneuver_goal) const;
  DecisionType ObstaclesDecision(ManeuverGoal* maneuver_goal) const;
  int SelectLane() const;
  FollowLaneState() = default;
  FollowLaneState(const FollowLaneState &other);
  FollowLaneState &operator=(const FollowLaneState &other);
 private:
  std::shared_ptr<ReferenceLine> reference_line_;

};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_FOLLOW_LANE_STATE_HPP_
