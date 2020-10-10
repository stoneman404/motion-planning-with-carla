#include "motion_planner/frenet_lattice_planner/frenet_lattice_planner.hpp"
#include "motion_planner/frenet_lattice_planner/trajectory_selector.hpp"
#include "motion_planner/frenet_lattice_planner/end_condition_sampler.hpp"

namespace planning {

bool FrenetLatticePlanner::Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                   const std::list<std::shared_ptr<ReferenceLine>> &reference_lines,
                                   const ManeuverGoal &maneuver_goal,
                                   std::shared_ptr<planning_msgs::Trajectory> pub_trajectory) {
  if (reference_lines.empty()) {
    ROS_FATAL("[FrenetLatticePlanner::Process]: No reference line provided");
    return false;
  }
  if (pub_trajectory == nullptr) {
    ROS_FATAL("[FrenetLatticePlanner::Process]: the pub_trajectory is nullptr");
    return false;
  }
  size_t index = 0;
  size_t failed_ref_plan_num = 0;
  std::vector<std::shared_ptr<planning_msgs::Trajectory>> traj_on_ref_line;
  for (const auto &ref_line : reference_lines) {
    bool result = Plan(init_trajectory_point, index, ref_line, maneuver_goal, &traj_on_ref_line);
    if (!result) {
      ROS_DEBUG("[FrenetLatticePlanner::Process], failed plan on reference line: %zu", index);
      failed_ref_plan_num++;
    }
    index++;
  }
  if (failed_ref_plan_num >= reference_lines.size()) {
    ROS_FATAL("[FrenetLatticePlanner::Process], the process is failed on every reference line");
    return false;
  }

  return true;
}

bool FrenetLatticePlanner::Plan(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                size_t index,
                                std::shared_ptr<ReferenceLine> ptr_ref_line,
                                const ManeuverGoal &maneuver_goal,
                                std::vector<std::shared_ptr<planning_msgs::Trajectory>> *traj_on_ref_line) const {

  return false;
}

void FrenetLatticePlanner::GenerateTrajectories(const ManeuverGoal &maneuver_goal,
                                                std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec,
                                                std::vector<std::shared_ptr<Polynomial>> *ptr_lat_traj_vec) const {

}

bool FrenetLatticePlanner::CombineTrajectories(std::shared_ptr<ReferenceLine> ptr_ref_line,
                                               const Polynomial &lon_traj_vec,
                                               const Polynomial &lat_traj_vec,
                                               std::shared_ptr<planning_msgs::Trajectory> ptr_combined_pub_traj) {
  return false;
}

void FrenetLatticePlanner::GenerateLatTrajectories(std::vector<std::shared_ptr<Polynomial>> *ptr_lat_traj_vec) const {

}

void FrenetLatticePlanner::GenerateLonTrajectories(const ManeuverGoal &maneuver_goal,
                                                   std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const {

}

void FrenetLatticePlanner::GenerateCruisingLonTrajectories(double cruise_speed,
                                                           std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const {

}

void FrenetLatticePlanner::GenerateStoppingLonTrajectories(double stop_s,
                                                           std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const {

}

void FrenetLatticePlanner::GenerateOvertakeAndFollowingLonTrajectories(std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const {

}

void FrenetLatticePlanner::GeneratePolynomialTrajectories(const std::array<double, 3> &init_condition,
                                                          const std::vector<std::pair<std::array<double, 3>,
                                                                                      double>> &end_conditions,
                                                          size_t order,
                                                          std::vector<std::shared_ptr<Polynomial>> *ptr_traj_vec) {

  if (ptr_traj_vec == nullptr) {
    ROS_FATAL("[FrenetLatticePlanner::GeneratePolynomialTrajectories], the pre_traj_vec is nullptr");
    return;
  }
  if (end_conditions.empty()) {
    ROS_FATAL("[FrenetLatticePlanner::GeneratePolynomialTrajectories], the end conditions vector is empty");
    return;
  }
  ROS_DEBUG("[FrenetLatticePlanner::GeneratePolynomialTrajectories], the end conditions vector's size: %zu",
            end_conditions.size());
  ptr_traj_vec->reserve(ptr_traj_vec->size() + end_conditions.size());
  switch (order) {
    case 4: {
      for (const auto &end_condition : end_conditions) {
        auto ptr_trajectory =
            std::make_shared<QuarticPolynomial>(init_condition[0], init_condition[1], init_condition[2],
                                                end_condition.first[1], end_condition.first[2], end_condition.second);
        ptr_traj_vec->push_back(ptr_trajectory);
      }
      break;
    }
    case 5: {
      for (const auto &end_condition : end_conditions) {
        auto ptr_trajectory =
            std::make_shared<QuinticPolynomial>(init_condition, end_condition.first, end_condition.second);
        ptr_traj_vec->push_back(ptr_trajectory);
        break;
      }
      break;
    }
    default:break;
  }
}

}