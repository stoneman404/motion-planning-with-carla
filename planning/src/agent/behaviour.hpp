#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
#include <planning_msgs/Trajectory.h>
#include "unordered_map"
#include "reference_line/reference_line.hpp"

namespace planning {
enum class LateralBehaviour : uint8_t {
  kUndefined = 0u,
  kLaneKeeping = 1u,
  kLaneChangeLeft = 2u,
  kLaneChangeRight = 3u
};
enum class LongitudinalBehaviour : uint8_t {
  kMaintain = 1u,
  kAccelate = 2u,
  kDecelate = 3u,
  kStopping = 4u
};

struct ProbDistributeOfLatBehaviour {
  void SetEntry(const LateralBehaviour &behaviour, double prob) {
    probs[behaviour] = prob;
  }

  bool GetMaxProbBehaviour(LateralBehaviour &behaviour) const {
    double max_prob = -1;
    LateralBehaviour max_behaviour;
    for (const auto &entry : probs) {
      if (entry.second > max_prob) {
        max_prob = entry.second;
        max_behaviour = entry.first;
      }
    }
    behaviour = max_behaviour;
    return true;
  }

  std::unordered_map<LateralBehaviour, double> probs{
      {LateralBehaviour::kLaneKeeping, 0.0},
      {LateralBehaviour::kLaneChangeLeft, 0.0},
      {LateralBehaviour::kLaneChangeRight, 0.0}};

};

struct Behaviour {
  LateralBehaviour lateral_behaviour;
  LongitudinalBehaviour longitudinal_behaviour;
//  ReferenceLine reference_line;
  double desired_velocity_{0.0};
  std::vector<std::unordered_map<int, planning_msgs::Trajectory>> surround_trajs;
  std::vector<LateralBehaviour> forward_behaviours;
  std::pair<ReferenceLine, planning_msgs::Trajectory> forward_trajs;

};

}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
