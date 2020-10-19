#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_STATE_HPP_
#include "maneuver_planner.hpp"
#include "planning_context.hpp"
#include <memory>
namespace planning {
class ManeuverPlanner;
class State {

 public:
  State() = default;
  virtual bool Enter(ManeuverPlanner *maneuver_planner) = 0;
  virtual ManeuverStatus Execute(ManeuverPlanner *maneuver_planner) = 0;
  virtual void Exit(ManeuverPlanner *maneuver_planner) = 0;
  virtual std::string Name() const = 0;
  virtual State *NextState(ManeuverPlanner *maneuver_planner) const = 0;
  virtual ~State() = default;

 protected:

  /**
   * @brief: get clear distance at certain lane
   * @param[in] lane_offset
   * @param[in] reference_line
   * @param[out] forward_clear_distance
   * @param[out] backward_clear_distance
   * @param[out] forward_obstacle_id
   * @param[out] backward_obstacle_id
   */
  virtual void GetLaneClearDistance(int lane_offset,
                                    std::shared_ptr<ReferenceLine> reference_line,
                                    double *forward_clear_distance,
                                    double *backward_clear_distance,
                                    int *forward_obstacle_id,
                                    int *backward_obstacle_id) const;

  virtual bool IsOnLane(const double ego_s, const double ego_d, const std::shared_ptr<ReferenceLine> &ref_line) const {
    double left_width = 0, right_width = 0;
    ref_line->GetLaneWidth(ego_s, &left_width, &right_width);
    if (ego_d < left_width && ego_d > -right_width) {
      return true;
    }
    return false;
  }

  /**
   * @brief: obstacle decision
   * @param[out] maneuver_goal
   */
  virtual void ObstacleDecision(ManeuverGoal *maneuver_goal) const = 0;

  /**
   * @brief: traffic light decision
   * @param[in] reference_line
   * @param[out] maneuver_goal
   */
  virtual void TrafficLightDecision(std::shared_ptr<ReferenceLine> reference_line,
                                    ManeuverGoal *maneuver_goal) const;

  /**
   * @brief: combine the obstacle and traffic light decision
   * @param[in] traffic_light_maneuver
   * @param[in] obstacle_maneuver
   * @return : the combined maneuver
   */
  virtual ManeuverGoal CombineManeuver(const ManeuverGoal &traffic_light_maneuver,
                                       const ManeuverGoal &obstacle_maneuver) const;
};
}

#endif
