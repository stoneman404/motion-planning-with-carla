
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
  virtual bool Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                       const std::list<std::shared_ptr<ReferenceLine>> &reference_lines,
                       planning_msgs::Trajectory::ConstPtr pub_trajectory) = 0;
};
}
#endif //
