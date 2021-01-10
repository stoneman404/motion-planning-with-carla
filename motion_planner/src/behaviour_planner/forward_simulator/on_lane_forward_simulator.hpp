#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_FORWARD_SIMULATOR_ON_LANE_FORWARD_SIMULATOR_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_FORWARD_SIMULATOR_ON_LANE_FORWARD_SIMULATOR_HPP_
#include <reference_line/reference_line.hpp>
#include <behaviour_planner/agent/behaviour.hpp>
#include <behaviour_planner/agent/agent.hpp>
namespace planning {

class OnLaneForwardSimulator {
 public:
  OnLaneForwardSimulator() = default;
  bool ForwardOneStep(const Agent &agent,
                      const SimulationParams &params,
                      const ReferenceLine &reference_line,
                      const Agent &leading_agent,
                      double sim_time_step,
                      planning_msgs::TrajectoryPoint &point);
 private:

  bool AgentMotionModel(const double s,
                        const double d,
                        const double vel,
                        const double acc,
                        const double approach_ratio,
                        const double dt,
                        const ReferenceLine &ref_lane,
                        planning_msgs::TrajectoryPoint &point);
  /**
   * @brief: get the agent's frenet state
   * @param[in] agent:
   * @param[in] reference_line
   * @param[out] s_conditions: s, \dot{s}, \ddot{s}
   * @param[out] d_conditions: l, l^{\prime}, l^{\prime}^{\prime}
   * @return: true if get agent's frenet state succeeds
   */
  static bool GetAgentFrenetState(const Agent &agent,
                                  const ReferenceLine &reference_line,
                                  std::array<double, 3> &s_conditions,
                                  std::array<double, 3> &d_conditions);

  /**
   * @brief: get agent's IDM longitudinal acceleration.
   * @param[in] ego_s_conditions: current agent frenet state
   * @param[in] reference_line:
   * @param[in] leading_agent: the leading agent,
   * @param[out] lon_acc: the lon acc for ego agent
   * @return : true: successful
   */
  bool GetIDMLonAcc(const std::array<double, 3> &ego_s_conditions,
                    const ReferenceLine &reference_line,
                    const Agent &leading_agent,
                    double &lon_acc) const;

  static planning_msgs::PathPoint AgentStateToPathPoint(const vehicle_state::KinoDynamicState &kino_dynamic_state);

  static void AgentMotionModel(const std::array<double, 3> &s_conditions,
                               const std::array<double, 3> &d_conditions,
                               double lateral_approach_ratio, double lon_acc,
                               double delta_t,
                               std::array<double, 3> &next_s_conditions,
                               std::array<double, 3> &next_d_conditions);

  static void FrenetStateToTrajectoryPoint(const std::array<double, 3> &s_conditions,
                                           const std::array<double, 3> &d_conditions,
                                           const ReferenceLine &ref_line,
                                           planning_msgs::TrajectoryPoint &trajectory_point);
 private:
  SimulationParams params_;

};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_FORWARD_SIMULATOR_ON_LANE_FORWARD_SIMULATOR_HPP_
