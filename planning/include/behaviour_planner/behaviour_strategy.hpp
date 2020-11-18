
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_INCLUDE_BEHAVIOUR_PLANNER_BEHAVIOUR_STRATEGY_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_INCLUDE_BEHAVIOUR_PLANNER_BEHAVIOUR_STRATEGY_HPP_

#include <utility>
#include <vector>
#include <unordered_map>
#include <planning_msgs/Trajectory.h>
#include <reference_line/reference_line.hpp>

namespace planning {

enum class LateralBehaviour : uint8_t {
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

struct Behaviour {
  Behaviour() = default;
  ~Behaviour() = default;
  Behaviour(const LateralBehaviour &lateral_behaviour, const LongitudinalBehaviour &longitudinal_behaviour,
            const ReferenceLine &reference_line,
            std::vector<std::pair<ReferenceLine, planning_msgs::Trajectory>>  forward_trajs,
            std::vector<LateralBehaviour>  forward_behaviours,
            const double desired_velocity)
      : lat_behaviour_(lateral_behaviour),
        lon_behaviour_(longitudinal_behaviour),
        reference_line_(reference_line),
        forward_trajs_(std::move(forward_trajs)),
        forward_behaviours_(std::move(forward_behaviours)),
        desired_velocity_(desired_velocity) {}

  // lateral behaviour
  LateralBehaviour lat_behaviour_{};
  // longitudinal behaviour
  LongitudinalBehaviour lon_behaviour_{};
  // reference line
  ReferenceLine reference_line_{};
  // the longitudinal behaviour is described by forward_trajs.
  std::vector<std::pair<ReferenceLine, planning_msgs::Trajectory>> forward_trajs_{};
  std::vector<LateralBehaviour> forward_behaviours_{};
  double desired_velocity_{};
};

class BehaviourStrategy {
 public:
  BehaviourStrategy() = default;
  virtual ~BehaviourStrategy() = default;
  virtual bool Execute(const planning_msgs::TrajectoryPoint &init_point, Behaviour &behaviour) = 0;
};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_INCLUDE_BEHAVIOUR_PLANNER_BEHAVIOUR_STRATEGY_HPP_
