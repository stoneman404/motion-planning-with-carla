
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
#include <planning_msgs/TrajectoryPoint.h>
#include <planning_msgs/Trajectory.h>
#include "reference_line/reference_line.hpp"

namespace planning {
class TrajectoryPlanner {
 public:
  TrajectoryPlanner() = default;
  virtual ~TrajectoryPlanner() = default;
  /**
   *
   * @param init_trajectory_point: the init trajectory point.
   * @param maneuver_goal: maneuver goal
   * @param optimal_trajectory: the optimal trajectory
   * @param valid_trajectories: for visualization
   * @return
   */
  virtual bool Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                       const ManeuverGoal &maneuver_goal,
                       planning_msgs::Trajectory &optimal_trajectory,
                       std::vector<planning_msgs::Trajectory> *valid_trajectories) = 0;
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
