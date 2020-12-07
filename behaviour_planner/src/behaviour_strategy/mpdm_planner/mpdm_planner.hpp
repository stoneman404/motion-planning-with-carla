#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_MPDM_BEHAVIOUR_PLANNER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_MPDM_BEHAVIOUR_PLANNER_HPP_
#include <thread_pool/thread_pool.hpp>
#include "behaviour_strategy/behaviour_strategy.hpp"
#include "agent/behaviour.hpp"
#include "agent/agent.hpp"
#include "policy_decider.hpp"

namespace planning {
class BehaviourStrategy;
class MPDMPlanner : public BehaviourStrategy {
 public:
  MPDMPlanner() = default;
  explicit MPDMPlanner(const ros::NodeHandle &nh,
                       const PolicySimulateConfig &config,
                       common::ThreadPool *thread_pool);
  ~MPDMPlanner() override = default;
  bool Execute(Behaviour &behaviour) override;
  void SetAgentSet(const std::unordered_map<int, Agent> &agent_set) override;

 private:

  /**
   * @brief: update ego agent's available policies, e.g. lane keeping or lane change left and lane change right,
   * as well as the corresponding reference lines.,
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
   * @brief: get reference line
   * @param[in] current_pose: current pose in world frame
   * @param[in] lateral_behaviour: lateral behaviour, e.g. LANE_CHANGE_RIGHT, will given the right ref lane
   * @param[in] forward_length
   * @param[in] backward_length
   * @param[in] need_to_smooth
   * @param[out] ref_line
   * @return
   */
  bool GetReferenceLane(const geometry_msgs::Pose &current_pose,
                        const LateralBehaviour &lateral_behaviour,
                        bool need_to_smooth,
                        std::shared_ptr<ReferenceLine> &ref_line);

 private:

  ros::NodeHandle nh_;
  std::unique_ptr<PolicyDecider> policy_decider_;
  common::ThreadPool *thread_pool_{};
  Agent ego_agent_;
  Behaviour behavior_;
  std::unordered_map<LateralBehaviour, std::shared_ptr<ReferenceLine>> available_policies_with_ref_lanes_;
  std::unordered_map<int, Agent> behaviour_agent_set_;
  ros::ServiceClient route_service_client_;

};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_BEHAVIOUR_PLANNING_BEHAVIOUR_PLANNER_HPP_
