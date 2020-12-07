#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
#include <planning_msgs/Trajectory.h>
#include "unordered_map"
#include "reference_line/reference_line.hpp"

namespace planning {
enum class LateralBehaviour : uint8_t {
  UNDEFINED = 0u,
  LANE_KEEPING = 1u,
  LANE_CHANGE_LEFT = 2u,
  LANE_CHANGE_RIGHT = 3u
};
enum class LongitudinalBehaviour : uint8_t {
  MAINTAIN = 1u,
  ACCELERATE = 2u,
  DECELERATE = 3u,
  STOPPING = 4u
};

struct EnumClassHash {
  template<class T>
  size_t operator()(T t) const {
    return static_cast<size_t>(t);
  }
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

  std::unordered_map<LateralBehaviour, double, EnumClassHash> probs{
      {LateralBehaviour::LANE_KEEPING, 0.0},
      {LateralBehaviour::LANE_CHANGE_LEFT, 0.0},
      {LateralBehaviour::LANE_CHANGE_RIGHT, 0.0},
      {LateralBehaviour::UNDEFINED, 0.0}};

};

struct Behaviour {
  std::vector<std::pair<LateralBehaviour, std::shared_ptr<ReferenceLine>>> forward_behaviours;
  std::vector<planning_msgs::Trajectory> forward_trajs;
  LateralBehaviour lateral_behaviour;
  LongitudinalBehaviour longitudinal_behaviour;
  std::vector<std::unordered_map<int, planning_msgs::Trajectory>> surrounding_trajs;
};

}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
