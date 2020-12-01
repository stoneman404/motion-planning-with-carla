
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_FORWARD_SIMULATOR_ON_LANE_FORWARD_SIMULATOR_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_FORWARD_SIMULATOR_ON_LANE_FORWARD_SIMULATOR_HPP_
#include <reference_line/reference_line.hpp>
#include <agent/behaviour.hpp>
#include <agent/agent.hpp>
namespace planning {
struct IDMParams {
  double desired_velocity{}; // v0
  double safe_time_headway{}; // T
  double max_acc{}; // a
  double max_decel{}; // b
  double acc_exponet{}; // /delta
  double s0{}; //jam_distance
  double s1{}; // jam distance
  double leading_vehicle_length_{};
};

struct SimulationParams {
  IDMParams idm_params;
  double max_default_lat_vel = 0.5;
  double lat_vel_ratio = 0.17;
  double lat_offset_threshold = 0.6;
};

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
                    double *lon_acc) const;

  static planning_msgs::PathPoint AgentStateToPathPoint(
      const vehicle_state::KinoDynamicState &kino_dynamic_state);

  void AgentMotionModel(const std::array<double, 3> &s_conditions,
                        const std::array<double, 3> &d_conditions,
                        const double v_lat, const double lon_acc,
                        const double delta_t,
                        std::array<double, 3> &next_s_conditions,
                        std::array<double, 3> &next_d_conditions) const;
  static void FrenetStateToTrajectoryPoint(const std::array<double, 3> &s_conditions,
                                           const std::array<double, 3> &d_conditions,
                                           const ReferenceLine &ref_line,
                                           planning_msgs::TrajectoryPoint &trajectory_point);
 private:
  SimulationParams params_;

};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_FORWARD_SIMULATOR_ON_LANE_FORWARD_SIMULATOR_HPP_
