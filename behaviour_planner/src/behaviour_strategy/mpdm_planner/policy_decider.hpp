#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_BEHAVIOUR_PLANNER_MPDM_PLANNER_POLICY_DECIDER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_BEHAVIOUR_PLANNER_MPDM_PLANNER_POLICY_DECIDER_HPP_
#include "agent/agent.hpp"
#include "agent/behaviour.hpp"
#include "forward_simulator/on_lane_forward_simulator.hpp"
#include <planning_msgs/Trajectory.h>

namespace planning {
struct PolicySimulateConfig {
  double desired_vel{8.333};
  double max_lat_acc{0.8};
  double sim_horizon_{10.0};
  double sim_step_{0.25};
  double safe_time_headway{0.5}; // T
  double max_acc{1.75}; // a
  double max_decel{0.8}; // b
  double acc_exponet{4.0}; // /delta
  double s0{2.0}; // jam distance
  double s1{0.0}; // jam distance
  double default_lat_approach_ratio = 0.995;
  double cutting_in_lateral_approach_ratio = 0.95;
};

struct SimulateAgent {
  Agent agent;
  std::pair<LateralBehaviour, ReferenceLine> behaviour_lane_pair;
};

class PolicyDecider {
  using Policy = std::pair<LateralBehaviour, ReferenceLine>;
  using Policies = std::vector<Policy>;
//  using TrajectoryRefLanePair = std::pair<ReferenceLine, planning_msgs::Trajectory>;
  using SurroundingTrajectories = std::unordered_map<int, planning_msgs::Trajectory>;
 public:
  explicit PolicyDecider(const PolicySimulateConfig &config);
  bool PolicyDecision(const Agent &ego_agent,
                      const std::unordered_map<int, Agent> &agents_set,
                      const Policies &possible_policies,
                      Policy &best_policy,
                      Policies &forward_policies,
                      std::vector<SurroundingTrajectories> &forward_surrounding_trajectories,
                      std::vector<planning_msgs::Trajectory> &forward_trajectories);

 private:

  static bool GetSimulateAgentSet(const std::unordered_map<int, Agent> &agent_set,
                                  const Policies &possible_policies,
                                  std::unordered_map<int, SimulateAgent> &simulate_agent_set);
  bool SimulateEgoAgentPolicy(const std::unordered_map<int, SimulateAgent> &simulate_agent_set,
                              const Policy &policy,
                              planning_msgs::Trajectory &ego_traj,
                              SurroundingTrajectories &surrounding_trajs);
  bool OpenLoopSimForward(const Policy &policy,
                          const std::unordered_map<int, SimulateAgent> &simulate_agents,
                          planning_msgs::Trajectory &ego_traj,
                          SurroundingTrajectories &surrounding_trajs);
  bool CloseLoopSimulate(const Policy &policy,
                         const std::unordered_map<int, SimulateAgent> &simulate_agents,
                         planning_msgs::Trajectory &ego_traj,
                         SurroundingTrajectories &surrounding_trajs);

  /**
   * @brief: simulate the agent with leading agent one step forward, if no leading agent,  leading_agent should be invalid
   * @param desired_vel: the current desired vel
   * @param agent: current agent
   * @param leading_agent: the leading agent, if no leading agent, make invalid
   * @param trajectory_point: one step forward trajectory point
   * @return
   */
  bool SimulateOneStepForAgent(double desired_vel,
                               const SimulateAgent &agent,
                               const Agent &leading_agent,
                               planning_msgs::TrajectoryPoint &trajectory_point);

  static bool NaivePredictionOnStepForAgent(const SimulateAgent &agent,
                                            double sim_step,
                                            planning_msgs::TrajectoryPoint &trajectory_point);

  /**
   * @brief: get desired speed consider reference lane's curvature and user defined reference velocity
   * @param xy
   * @param ref_lane
   * @return
   */
  double GetDesiredSpeed(const std::pair<double, double> &xy, const ReferenceLine &ref_lane) const;

  /**
   * @brief: get the leading agent on the \ref_lane
   * @param agent
   * @param ref_lane
   * @param agent_id : -1, no leading vehicle, otherwise , it is the id of leading agent.
   * @return true if normal,
   */
  static bool GetLeadingAgentOnRefLane(const Agent &agent,
                                       const std::unordered_map<int, SimulateAgent> &simulate_agents,
                                       const ReferenceLine &ref_lane,
                                       int &agent_id);

  bool EvaluateMultiPolicyTrajectories(const std::vector<Policy> &valid_policy,
                                       const std::vector<planning_msgs::Trajectory> &valid_forward_trajectory,
                                       const std::vector<std::unordered_map<int,
                                                                            planning_msgs::Trajectory>> &valid_surrounding_trajectories,
                                       Policy &winner_policy,
                                       planning_msgs::Trajectory &winner_forward_trajectory,
                                       double &winner_score);

  void EvaluateSinglePolicyTrajectory(const Policy &policy,
                                      const planning_msgs::Trajectory &trajectory,
                                      const std::unordered_map<int, planning_msgs::Trajectory> &surrounding_trajs,
                                      double *score) const;

  bool EvaluateSafetyCost(int ego_id, const planning_msgs::Trajectory &trajectory,
                          const std::pair<int, planning_msgs::Trajectory> &other_trajectory,
                          double *safety_cost) const;

 private:
  int ego_id_{};
  PolicySimulateConfig config_;
  std::unique_ptr<OnLaneForwardSimulator> forward_simulator_;
  std::unordered_map<int, Agent> agent_set_;


};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_BEHAVIOUR_PLANNER_MPDM_PLANNER_POLICY_DECIDER_HPP_
