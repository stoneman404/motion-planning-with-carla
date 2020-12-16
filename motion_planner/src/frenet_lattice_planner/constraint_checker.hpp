
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNING_INCLUDE_MOTION_PLANNER_FRENET_LATTICE_PLANNER_CONSTRAINT_CHECKER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNING_INCLUDE_MOTION_PLANNER_FRENET_LATTICE_PLANNER_CONSTRAINT_CHECKER_HPP_
#include <planning_msgs/Trajectory.h>
namespace planning {
class ConstraintChecker {
 public:
  enum class Result {
    VALID,
    LON_VELOCITY_OUT_OF_BOUND,
    LON_ACCELERATION_OUT_OF_BOUND,
    LON_JERK_OUT_OF_BOUND,
    LAT_VELOCITY_OUT_OF_BOUND,
    LAT_ACCELERATION_OUT_OF_BOUND,
    LAT_JERK_OUT_OF_BOUND,
    CURVATURE_OUT_OF_BOUND
  };
  ConstraintChecker() = default;
  ~ConstraintChecker() = default;

  static bool WithInRange(double value, double lower, double upper, double eps = 1e-2);
  static Result ValidTrajectory(const planning_msgs::Trajectory &trajectory);
};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNING_INCLUDE_MOTION_PLANNER_FRENET_LATTICE_PLANNER_CONSTRAINT_CHECKER_HPP_
