#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_MPDM_BEHAVIOUR_PLANNER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_MPDM_BEHAVIOUR_PLANNER_HPP_
#include <thread_pool/thread_pool.hpp>
#include "behaviour_strategy/behaviour_strategy.hpp"
#include "agent/behaviour.hpp"
#include "agent/agent.hpp"
#include "policy_decider.hpp"

namespace planning {

class MPDMPlanner : public BehaviourStrategy {
 public:

  MPDMPlanner() = default;
  explicit MPDMPlanner(const PolicySimulateConfig &config,
                       common::ThreadPool *thread_pool);
  ~MPDMPlanner() override = default;

  /**
   * @brief: Main Function
   * @param[out] behaviour
   * @param[in] reference_lines: reference lines.
   * @return: true if this procedure is successful, false otherwise
   */
  bool Execute(Behaviour &behaviour, const std::vector<ReferenceLine> &reference_lines) override;

  /**
   * @brief: set simulate agent set including ego agent.
   * @param[in] ego_id: id of host agent
   * @param[in] agent_set: the agent set
   */
  void SetAgentSet(int ego_id, const std::unordered_map<int, Agent> &agent_set) override;

 private:

  /**
   * @brief: get available behaviours and reference lines for host agent
   * @param[in] reference_lines: all reference lines
   * @return: true if this procedure is successful, false otherwise
   */
  bool UpdateAvailableBehaviourRefLanePairs(const std::vector<ReferenceLine> &reference_lines);

 private:
  bool update_agent_ = false;
  std::unique_ptr<PolicyDecider> policy_decider_;
  common::ThreadPool *thread_pool_{};
  Agent ego_agent_;
  Behaviour behavior_;
  std::unordered_map<LateralBehaviour, ReferenceLine, EnumClassHash> available_policies_with_ref_lanes_;
  std::unordered_map<int, Agent> behaviour_agent_set_;

};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_BEHAVIOUR_PLANNING_BEHAVIOUR_PLANNER_HPP_
