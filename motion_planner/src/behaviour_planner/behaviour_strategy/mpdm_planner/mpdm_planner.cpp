#include <tf/transform_datatypes.h>
#include <behaviour_planner/agent/behaviour.hpp>
#include <behaviour_planner/agent/agent.hpp>
#include "mpdm_planner.hpp"
#include "name/string_name.hpp"

namespace planning {
MPDMPlanner::MPDMPlanner(const PolicySimulateConfig &config,
                         common::ThreadPool *thread_pool)
    : thread_pool_(thread_pool) {
  behavior_.longitudinal_behaviour = LongitudinalBehaviour::STOPPING;
  behavior_.lateral_behaviour = LateralBehaviour::LANE_KEEPING;
  behavior_.forward_behaviours = decltype(behavior_.forward_behaviours)();
  behavior_.forward_trajs = decltype(behavior_.forward_trajs)();
  behavior_.ref_lane = ReferenceLine();
  policy_decider_ = std::make_unique<PolicyDecider>(config);
}

bool MPDMPlanner::Execute(const std::vector<ReferenceLine> &reference_lines, Behaviour &behaviour) {

  if (!update_agent_) {
    return false;
  }
  if (!UpdateAvailableBehaviourRefLanePairs(reference_lines)) {
    return false;
  }
  std::vector<std::pair<LateralBehaviour, ReferenceLine>> possible_policies;
  std::pair<LateralBehaviour, ReferenceLine> best_policy;
  std::vector<std::pair<LateralBehaviour, ReferenceLine>> forward_policies;
  std::vector<std::unordered_map<int, planning_msgs::Trajectory>> surround_trajs;
  std::vector<planning_msgs::Trajectory> forward_trajectories;
  for (const auto &policy : available_policies_with_ref_lanes_) {
    possible_policies.emplace_back(policy);
  }

  if (!policy_decider_->PolicyDecision(ego_agent_,
                                       behaviour_agent_set_,
                                       possible_policies,
                                       best_policy,
                                       forward_policies,
                                       surround_trajs,
                                       forward_trajectories)) {
    return false;
  }

  behaviour.lateral_behaviour = best_policy.first;
  behaviour.ref_lane = best_policy.second;
  behaviour.forward_behaviours = forward_policies;
  behaviour.forward_trajs = forward_trajectories;
  behaviour.surrounding_trajs = surround_trajs;
  behavior_ = behaviour;
  return true;
}

void MPDMPlanner::SetAgentSet(int ego_id, const std::unordered_map<int, Agent> &agent_set) {
  behaviour_agent_set_ = agent_set;
  ego_agent_ = behaviour_agent_set_.at(ego_id);
  update_agent_ = true;
}

bool MPDMPlanner::UpdateAvailableBehaviourRefLanePairs(const std::vector<ReferenceLine> &reference_lines) {
  if (reference_lines.empty()) {
    return false;
  }
  available_policies_with_ref_lanes_.clear();
  const double ego_x = ego_agent_.state().x;
  const double ego_y = ego_agent_.state().y;
  constexpr double kDefaultHalfLaneWidth = 1.75;
  constexpr double kOffset = 0.3;
  for (const auto &ref_line : reference_lines) {
    common::SLPoint sl_point;
    if (!ref_line.XYToSL(ego_x, ego_y, &sl_point)) {
      continue;
    }
    if (ref_line.IsOnLane(sl_point)) {
      available_policies_with_ref_lanes_.emplace(LateralBehaviour::LANE_KEEPING, ref_line);
      continue;
    }

    if (sl_point.l < -3.0 * kDefaultHalfLaneWidth || sl_point.l > 3.0 * kDefaultHalfLaneWidth) {
      continue;
    }
    if (sl_point.l > -2.0 * kDefaultHalfLaneWidth - kOffset && sl_point.l < -1.0 * kDefaultHalfLaneWidth) {
      available_policies_with_ref_lanes_.emplace(LateralBehaviour::LANE_CHANGE_LEFT, ref_line);
      continue;
    }
    if (sl_point.l < 2.0 * kDefaultHalfLaneWidth + kOffset && sl_point.l > kDefaultHalfLaneWidth) {
      available_policies_with_ref_lanes_.emplace(LateralBehaviour::LANE_CHANGE_RIGHT, ref_line);
      continue;
    }
  }
  if (available_policies_with_ref_lanes_.empty()) {
    return false;
  }
  return true;
}



}