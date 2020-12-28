#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
#include <planning_msgs/Trajectory.h>
#include <vehicle_state/kinodynamic_state.hpp>
#include <queue>
#include <utility>
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
  bool GetKthMaxProbBehavioursAndLanes(
      uint32_t k,
      std::vector<std::pair<LateralBehaviour, std::shared_ptr<ReferenceLine>>> &behaviour_lane_pairs);
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

class ReferenceInfo {
 public:
  ReferenceInfo() = default;
  ~ReferenceInfo() = default;
  ReferenceInfo(double max_curvature,
                double deviation_weight,
                double heading_weight,
                double length_weight,
                std::vector<planning_msgs::WayPoint> main_lane,
                std::vector<std::vector<planning_msgs::WayPoint>> left_lanes,
                std::vector<std::vector<planning_msgs::WayPoint>> right_lanes);
  /**
   * @brief:
   *
   * @param[out] ref_lanes, the reference line vector, ref_lanes[0]: the main reference line, if the ref_lanes.size() > 1,
   * then there're changeable lanes, right lane or/and left lane
   * @return: true if this procedure is successful.
   */
  bool GetReferenceLines(const vehicle_state::KinoDynamicState &vehicle_state,
                         double lookahead_distance,
                         double lookback_distance,
                         bool smooth,
                         std::vector<std::shared_ptr<ReferenceLine>> &ref_lanes);

  /**
   *
   * @param vehicle_state
   * @param lane
   * @param lookahead_distance
   * @param lookback_distance
   * @param smooth
   * @param max_curvature
   * @param deviation_weight
   * @param heading_weight
   * @param length_weight
   * @param ref_lane
   * @return
   */
  static bool RetriveReferenceLineFromRoute(std::shared_ptr<ReferenceLine> &ref_lane,
                                            const vehicle_state::KinoDynamicState &vehicle_state,
                                            const std::vector<planning_msgs::WayPoint> &lane,
                                            double lookahead_distance,
                                            double lookback_distance,
                                            bool smooth = false,
                                            double max_curvature = 0.0,
                                            double deviation_weight = 0.0,
                                            double heading_weight = 0.0,
                                            double length_weight = 0.0);

 private:

  static bool HasOverLapWithRefLane(const std::shared_ptr<ReferenceLine> &ref_lane,
                                    std::vector<planning_msgs::WayPoint> &waypoints,
                                    double *overlap_start_s,
                                    double *overlap_end_s);

 private:
  double reference_smooth_max_curvature_;
  double reference_smooth_deviation_weight_;
  double reference_smooth_heading_weight_;
  double reference_smooth_length_weight_;
  std::vector<planning_msgs::WayPoint> main_lane_;
  std::vector<std::vector<planning_msgs::WayPoint>> left_lanes_;
  std::vector<std::vector<planning_msgs::WayPoint>> right_lanes_;

};

}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_ACTION_BEHAVIOUR_HPP_
