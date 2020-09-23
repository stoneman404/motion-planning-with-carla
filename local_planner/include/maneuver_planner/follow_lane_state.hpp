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

  /**
   * @brief: enter action
   * @param maneuver_planner
   * @return
   */
  bool Enter(ManeuverPlanner *maneuver_planner) override;

  /**
   * @brief: execute action
   * @param maneuver_planner
   * @return
   */
  bool Execute(ManeuverPlanner *maneuver_planner) override;

  /**
   * @brief: exit action
   * @param maneuver_planner
   */
  void Exit(ManeuverPlanner *maneuver_planner) override;

  /**
   * @brief: get the name of current state
   * @return the name of this state
   */
  std::string Name() const override;

  /**
   * @brief:
   * @return
   */
  static State &Instance();

  /**
   *
   * @param[in, out] maneuver_planner
   * @return : return the next state if current state transition
   */
  State *NextState(ManeuverPlanner *maneuver_planner) const override;

 private:

  /**
   * @brief: calc the target lane clear distance,
   * both front and rear clear distance as well as leading and following obstacle id
   * @param[in] lane_offset : lane offset, -1: offset to left, +1 offset to rigth, 0 no offset
   * @param[out] forward_clear_distance: the front clear distance at thr target lane
   * @param[out] backward_clear_distance: the rear clear distance at the target lane
   * @param[out] forward_obstacle_id : the front obstacle id, -1: means has no front obstacle
   * @param[out] backward_obstacle_id: the rear obstacle id,  -1: means has no rear obstacle
   */
  void GetLaneClearDistance(int lane_offset,
                            double *forward_clear_distance,
                            double *backward_clear_distance,
                            int *forward_obstacle_id,
                            int *backward_obstacle_id) const;

  /**
   * @brief: the traffic light decision
   * @param[in,out] maneuver_goal: the maneuver goal determined by traffic decision
   * @return : the decision type
   */
  void TrafficLightsDecision(ManeuverGoal *maneuver_goal) const;

  /**
   * @brief: the obstacle decision
   * @param[in, out] maneuver_goal: the maneuver goal determined by obstacles
   * @return : the decision type
   */
  void ObstaclesDecision(ManeuverGoal *maneuver_goal) const;

  /**
   * @brief: change lane decision process, used by obstacle decision
   * @param current_lane_forward_clear_distance
   * @param current_lane_backward_clear_distance
   * @param current_lane_forward_obstacle_id
   * @param current_lane_backward_obstacle_id
   * @param incoming_way_point
   * @param maneuver_goal
   */
  void ChangeLaneDecision(double ego_s,
                          double current_lane_forward_clear_distance,
                          double current_lane_backward_clear_distance,
                          int current_lane_forward_obstacle_id,
                          int current_lane_backward_obstacle_id,
                          const planning_msgs::WayPoint &incoming_way_point,
                          ManeuverGoal *maneuver_goal) const;

  /**
   * @brief: select the target lane
   * @return : the target lane id
   */
  int SelectLane() const;
  FollowLaneState() = default;
  FollowLaneState(const FollowLaneState &other);
  FollowLaneState &operator=(const FollowLaneState &other);
 private:
  std::shared_ptr<ReferenceLine> reference_line_;

};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_FOLLOW_LANE_STATE_HPP_
