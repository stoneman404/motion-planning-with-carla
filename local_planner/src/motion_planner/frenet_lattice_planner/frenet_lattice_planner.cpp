#include "motion_planner/frenet_lattice_planner/frenet_lattice_planner.hpp"
#include "motion_planner/frenet_lattice_planner/trajectory_selector.hpp"
#include "motion_planner/frenet_lattice_planner/end_condition_sampler.hpp"
namespace planning {

bool FrenetLatticePlanner::Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                   const std::list<std::shared_ptr<ReferenceLine>> &reference_lines,
                                   const ManeuverGoal &maneuver_goal,
                                   planning_msgs::Trajectory::ConstPtr pub_trajectory) {
  if (reference_lines.empty()) {
    ROS_FATAL("[FrenetLatticePlanner::Process:] No reference line provided");
    return false;
  }
  if (pub_trajectory == nullptr) {
    ROS_FATAL("[FrenetLatticePlanner::Process:], the pub_trajectory is nullptr");
    return false;
  }

  return false;
}
}