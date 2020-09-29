#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_FRENET_LATTICE_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_FRENET_LATTICE_PLANNER_HPP_
#include "planner/trajectory_planner.hpp"
#include <planning_msgs/TrajectoryPoint.h>
#include "planning_context.hpp"
#include "st_graph.hpp"
namespace planning {

class FrenetLatticePlanner : public TrajectoryPlanner {
 public:
  FrenetLatticePlanner() = default;
  ~FrenetLatticePlanner() override = default;
  bool Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
               const std::list<std::shared_ptr<ReferenceLine>> &reference_lines,
               const ManeuverGoal &maneuver_goal,
               planning_msgs::Trajectory::ConstPtr pub_trajectory) override;
};

}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_FRENET_LATTICE_PLANNER_HPP_