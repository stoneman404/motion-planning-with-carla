#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_FORWARD_SIMULATOR_PURE_PURSUIT_CONTROL_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_FORWARD_SIMULATOR_PURE_PURSUIT_CONTROL_HPP_
#include <cmath>

namespace planning {
class PurePursuitControl {
 public:
  static bool CalculateDesiredSteer(const double wheel_base_len,
                                    const double angle_diff,
                                    const double lookahead_dist,
                                    double *steer);
};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_FORWARD_SIMULATOR_PURE_PURSUIT_CONTROL_HPP_
