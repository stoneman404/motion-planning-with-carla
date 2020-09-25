#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_FOLLOW_LANE_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_FOLLOW_LANE_STATE_HPP_
#include "state.hpp"
#include <vector>
#include "planning_context.hpp"
#include "change_left_lane_state.hpp"
#include "change_right_lane_state.hpp"
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
   * @param[in, out] maneuver_planner
   * @return : return the next state if current state transition
   */
  State *NextState(ManeuverPlanner *maneuver_planner) const override;

 protected:
  /**
   * @brief: obstacle decision
   * @param maneuver_goal
   */
  void ObstacleDecision(ManeuverGoal *maneuver_goal) const override;

  /**
  * @brief: the traffic light decision
  * @param[in,out] maneuver_goal: the maneuver goal determined by traffic decision
  * @return : the decision type
  */
  void TrafficLightDecision(ManeuverGoal *maneuver_goal) const override;

 private:

  static ManeuverGoal CombineManeuver(const ManeuverGoal &traffic_maneuver, const ManeuverGoal &obstacle_maneuver);

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
   *
   * @param ego_s
   * @param ego_vel
   * @param leading_velocity: [left_lane, current_lane, right_lane]
   * @param following_velocity [left_lane, current_lane, right_lane]
   * @param leading_clear_distance [left_lane, current_lane, right_lane]
   * @param following_clear_distance [left_lane, current_lane, right_lane]
   * @return
   */
  static int SelectLane(double ego_s, double ego_vel,
                        const std::vector<double> &leading_velocity,
                        const std::vector<double> &following_velocity,
                        const std::vector<double> &leading_clear_distance,
                        const std::vector<double> &following_clear_distance);

  static double SafetyCost(double leading_vel,
                           double following_vel,
                           double leading_clear_distance,
                           double following_clear_distance);

  static double EfficiencyCost(double target_vel,
                               double leading_vel,
                               double max_vel);

  static double ComfortCost(double ego_vel, double leading_vel, double forward_clear_distance);

  FollowLaneState() = default;
  FollowLaneState(const FollowLaneState &other);
  FollowLaneState &operator=(const FollowLaneState &other);
 private:
  std::shared_ptr<ReferenceLine> reference_line_;

};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_FOLLOW_LANE_STATE_HPP_
