
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
#include <planning_msgs/TrajectoryPoint.h>
#include <planning_msgs/Trajectory.h>
#include <planning_msgs/LongitudinalBehaviour.h>
#include <planning_msgs/LateralBehaviour.h>
#include "reference_line/reference_line.hpp"
#include "obstacle_manager/obstacle.hpp"
#include "planning_config.hpp"

namespace planning {

class TrajectoryPlanner {
 public:
  TrajectoryPlanner() = default;
  virtual ~TrajectoryPlanner() = default;
  /**
   *
   * @param init_trajectory_point: the init trajectory point.
   * @param planning_targets: planning_targets
   * @param optimal_trajectory: the optimal trajectory
   * @param valid_trajectories: for visualization
   * @return
   */
  virtual bool Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                       const std::vector<PlanningTarget> &planning_targets,
                       planning_msgs::Trajectory &optimal_trajectory,
                       std::vector<planning_msgs::Trajectory> *valid_trajectories) = 0;
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
