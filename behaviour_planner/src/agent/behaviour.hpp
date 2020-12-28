#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
#include <planning_msgs/Trajectory.h>
#include <queue>
#include "unordered_map"
#include "reference_line/reference_line.hpp"

namespace planning {
enum class LateralBehaviour : uint8_t {
  UNDEFINED = 0u,
  LANE_KEEPING = 1u,
  LANE_CHANGE_LEFT = 2u,
  LANE_CHANGE_RIGHT = 3u,
  TURN_RIGHT = 4u,
  TURN_LEFT = 5u,
  GO_STRAIGHT = 6u

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

class ProbDistributeOfLatBehaviour {
 public:
  using BehaviourProbPair = std::pair<std::pair<LateralBehaviour, std::shared_ptr<ReferenceLine>>, double>;
  using BehaviourProbPairVector = std::vector<BehaviourProbPair>;

  ProbDistributeOfLatBehaviour() = default;
  ~ProbDistributeOfLatBehaviour() = default;

  /**
   * @brief set lateral behaviour's prob
   * @param behaviour
   * @param prob
   */
  void SetEntry(const LateralBehaviour &behaviour, double prob, const std::shared_ptr<ReferenceLine> &lane);

  /**
 * @brief: get the most likely k-th behaviour and lanes
 * @note: this method will pop the i<k th behaviour and lanes
 * @param behaviour_lane_pairs
 * @return
 */
  bool GetKthMaxProbBehavioursAndLanes(uint32_t k, std::vector<std::pair<LateralBehaviour,
                                                                         std::shared_ptr<ReferenceLine>>> &behaviour_lane_pairs);
  /**
   * @brief get the most likely behaviour
   * @param behaviour
   * @return
   */
  bool GetMaxProbBehaviour(LateralBehaviour &behaviour) const;

  /**
   * @brief get the most likely behaviour and it's lane
   * @param behaviour
   * @param lane
   * @return
   */
  bool GetMaxProbBehaviourAndLane(LateralBehaviour &behaviour, std::shared_ptr<ReferenceLine> &lane) const;

 private:
  struct cmp : public std::binary_function<const BehaviourProbPair &, const BehaviourProbPair, bool> {
    bool operator()(const BehaviourProbPair &left, const BehaviourProbPair &right) {
      return left.second < right.second;
    }
  };
  std::priority_queue<BehaviourProbPair, BehaviourProbPairVector, cmp> prob_dist_queue_;
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
