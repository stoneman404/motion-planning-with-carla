
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
#include <planning_msgs/TrajectoryPoint.h>
#include <planning_msgs/Trajectory.h>
#include "reference_line/reference_line.hpp"
#include "planning_context.hpp"

namespace planning {
class TrajectoryPlanner {
 public:
  TrajectoryPlanner() = default;
  virtual ~TrajectoryPlanner() = default;
  virtual bool Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                       const ManeuverGoal &maneuver_goal,
                       std::shared_ptr<planning_msgs::Trajectory> pub_trajectory) = 0;
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
