
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_TRAJECTORY_PLANNER_HPP_
namespace planning {
class TrajectoryPlanner {
 public:
  TrajectoryPlanner() = default;
  virtual ~TrajectoryPlanner() = default;
  virtual bool Execute() = 0;
};
}
#endif //
