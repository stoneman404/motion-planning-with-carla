#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_MPDM_BEHAVIOUR_PLANNER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_MPDM_BEHAVIOUR_PLANNER_HPP_
#include <thread_pool/thread_pool.hpp>
#include "behaviour_planner/behaviour_strategy.hpp"
#include "agent/behaviour.hpp"
#include "agent/agent.hpp"

namespace planning {
class BehaviourStrategy;
class MPDMPlanner : public BehaviourStrategy {
 public:
  MPDMPlanner() = default;
  explicit MPDMPlanner(const ros::NodeHandle &nh, common::ThreadPool *thread_pool = nullptr);
  ~MPDMPlanner() override = default;
  bool Execute(const planning_msgs::TrajectoryPoint &init_point, Behaviour &behaviour) override;

 private:

  /**
   * @brief: the mpdm decision process
   * @param[out] mpdm_decision: the decision decided by mpdm process
   * @param[out] desired_velocity: the desired velocity decided by mpdm process
   * @return
   */
  bool MultiPoliciesDecision(Behaviour &mpdm_decision, double &desired_velocity);

  /**
   * @brief: simulate ego agent's behaviour reaction with other agents
   * @param policy: the policy for ego vehicle
   * @param[out] ego_traj
   * @param surrounding_trajs
   * @return
   */
  bool SimulateEgoAgentPolicy(const LateralBehaviour &policy, planning_msgs::Trajectory &ego_traj,
                              std::pair<int, planning_msgs::Trajectory> &surrounding_trajs);

  bool CloseLoopSimForward(const LateralBehaviour &policy, planning_msgs::Trajectory &ego_traj,
                           std::pair<int, planning_msgs::Trajectory> &surrounding_trajs);

  bool OpenLoopSimForward(const LateralBehaviour &policy, planning_msgs::Trajectory &ego_traj,
                          std::pair<int, planning_msgs::Trajectory> &surrounding_trajs);

  /**
   * @brief: update the agents the most likely behaviours and corresponding reference line
   * @param agents: the agent vector
   */
  void UpdateAgentsMostLikelyPolicies(const std::vector<Agent> &agents);

  /**
   * @brief: update ego agent's available policies, e.g. lane keeping or lane change left and lane change right,
   * as well as get the corresponding reference lines., e.g. the left lane and the right lane for lane change left and
   * lane change right.
   * @return true if success
   */
  bool UpdateAvailablePoliciesAndRefLanes();

  /**
   * @brief: update agent most likely behaviour and reference line.
   * @param[in,out] agent: the agent to be updated
   * @return : true if success
   */
  bool GetAgentMostLikelyPolicyAndReferenceLane(Agent &agent);

  /**
   * @brief: get the route
   * @param[in] start the start pose
   * @param[in] destination: the end pose
   * @param[in] lateral_behaviour: the lateral behaviour
   * @param[out] route_response: the route response
   * @return
   */
  bool GetRoute(const geometry_msgs::Pose &start,
                const geometry_msgs::Pose &destination,
                const LateralBehaviour &lateral_behaviour,
                planning_srvs::RoutePlanServiceResponse &route_response);

  /**
   * @brief: get
   * @param current_pose
   * @param lateral_behaviour
   * @param forward_length
   * @param backward_length
   * @param need_to_smooth
   * @param ref_line
   * @return
   */
  bool GetReferenceLane(const geometry_msgs::Pose &current_pose,
                        const LateralBehaviour &lateral_behaviour,
                        double forward_length, double backward_length,
                        bool need_to_smooth,
                        std::shared_ptr<ReferenceLine> &ref_line);

 private:
  ros::NodeHandle nh_;
  common::ThreadPool *thread_pool_{};
  Agent ego_agent_;
  Behaviour behavior_;
  std::unordered_map<LateralBehaviour, std::shared_ptr<ReferenceLine>> available_policies_with_ref_lanes_;
  std::unordered_map<int, Agent> behaviour_agent_set_;
  ros::ServiceClient route_service_client_;
  std::vector<LateralBehaviour> forward_behaviours_;
  std::vector<std::pair<ReferenceLine, planning_msgs::Trajectory>> forward_trajectories_;
  std::vector<std::pair<int, planning_msgs::Trajectory>> surrounding_trajectories_;
};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_BEHAVIOUR_PLANNING_BEHAVIOUR_PLANNER_HPP_
