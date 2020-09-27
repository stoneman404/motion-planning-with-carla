#include "planner/frenet_lattice_planner/frenet_lattice_planner.hpp"
#include "planner/frenet_lattice_planner/trajectory_evaluator.hpp"
#include "planner/frenet_lattice_planner/end_condition_sampler.hpp"
namespace planning {

bool FrenetLatticePlanner::Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                   const std::list<std::shared_ptr<ReferenceLine>> &reference_lines,
                                   planning_msgs::Trajectory::ConstPtr pub_trajectory) {
  return false;
}
}