#include "behaviour.hpp"
namespace planning {

void ProbDistributeOfLatBehaviour::SetEntry(const LateralBehaviour &behaviour,
                                            double prob,
                                            const std::shared_ptr<ReferenceLine> &lane) {

  prob_dist_queue_.emplace(std::make_pair(behaviour, lane), prob);
}

bool ProbDistributeOfLatBehaviour::GetMaxProbBehaviour(LateralBehaviour &behaviour) const {
  if (prob_dist_queue_.empty()) {
    return false;
  }
  behaviour = prob_dist_queue_.top().first.first;
  return true;
}

bool ProbDistributeOfLatBehaviour::GetMaxProbBehaviourAndLane(LateralBehaviour &behaviour,
                                                              std::shared_ptr<ReferenceLine> &lane) const {
  if (prob_dist_queue_.empty()) {
    return false;
  }
  behaviour = prob_dist_queue_.top().first.first;
  lane = prob_dist_queue_.top().first.second;
  return true;
}

bool ProbDistributeOfLatBehaviour::GetKthMaxProbBehavioursAndLanes(
    uint32_t k,
    std::vector<std::pair<LateralBehaviour, std::shared_ptr<ReferenceLine>>> &behaviour_lane_pairs) {
  if (k < 1) {
    return false;
  }
  if (prob_dist_queue_.empty()) {
    return false;
  }
  behaviour_lane_pairs.clear();
  uint32_t cnt = 0;
  while (!prob_dist_queue_.empty() && cnt < k) {
    auto behaviour_pair = prob_dist_queue_.top();
    behaviour_lane_pairs.emplace_back(behaviour_pair.first);
    prob_dist_queue_.pop();
    ++cnt;
  }
  return true;
}

}