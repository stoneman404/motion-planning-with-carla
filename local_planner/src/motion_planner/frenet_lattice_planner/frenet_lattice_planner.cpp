#include "motion_planner/frenet_lattice_planner/frenet_lattice_planner.hpp"
#include "motion_planner/frenet_lattice_planner/polynomial_trajectory_evaluator.hpp"
#include "motion_planner/frenet_lattice_planner/end_condition_sampler.hpp"
#include "coordinate_transformer.hpp"
#include "obstacle_filter/obstacle_filter.hpp"

namespace planning {

bool FrenetLatticePlanner::Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                   const ManeuverGoal &maneuver_goal,
                                   std::shared_ptr<planning_msgs::Trajectory> pub_trajectory) {
  if (maneuver_goal.maneuver_infos.empty()) {
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
  for (const auto &maneuver_info : maneuver_goal.maneuver_infos) {
//    auto traj_on_ref_line = std::make_shared<planning_msgs::Trajectory>();
    bool result = FrenetLatticePlanner::Plan(init_trajectory_point, index, maneuver_info, &traj_on_ref_line);
    if (!result) {
      ROS_DEBUG("[FrenetLatticePlanner::Process], failed plan on reference line: %zu", index);
      failed_ref_plan_num++;
    }
    index++;
  }
  if (failed_ref_plan_num >= maneuver_goal.maneuver_infos.size()) {
    ROS_FATAL("[FrenetLatticePlanner::Process], the process is failed on every reference line");
    return false;
  }

  return true;
}

bool FrenetLatticePlanner::Plan(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                size_t index, const ManeuverInfo &maneuver_info,
                                std::vector<std::shared_ptr<planning_msgs::Trajectory>> *trajs_on_ref_line) {
  if (trajs_on_ref_line == nullptr) {
    ROS_FATAL("[FrenetLatticePlanner::Plan] failed because the traj_on_ref_line is nullptr");
  }
  std::vector<std::shared_ptr<Polynomial>> lon_traj_vec;
  std::vector<std::shared_ptr<Polynomial>> lat_traj_vec;
  FrenetLatticePlanner::GenerateTrajectories(init_trajectory_point, maneuver_info, &lon_traj_vec, &lat_traj_vec);

  return false;
}

void FrenetLatticePlanner::GenerateTrajectories(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                                const ManeuverInfo &maneuver_info,
                                                std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec,
                                                std::vector<std::shared_ptr<Polynomial>> *ptr_lat_traj_vec) {

  const auto &ref_line = maneuver_info.ptr_ref_line;
  std::array<double, 3> init_s{};
  std::array<double, 3> init_d{};
  FrenetLatticePlanner::GetInitCondition(ref_line, init_trajectory_point, &init_s, &init_d);
  const auto &obstacles_map = ObstacleFilter::Instance().Obstacles();
  std::vector<std::shared_ptr<Obstacle>> obstacle_vec;
  obstacle_vec.reserve(obstacles_map.size());
  for (const auto &obstacle : obstacles_map) {
    obstacle_vec.push_back(obstacle.second);
  }
  auto st_graph = std::make_shared<STGraph>(obstacle_vec, ref_line,
                                            init_s[0],
                                            init_s[0] + PlanningConfig::Instance().max_lookahead_distance(),
                                            0.0, PlanningConfig::Instance().max_lookahead_time(),
                                            init_d);
  if (ptr_lat_traj_vec == nullptr || ptr_lon_traj_vec == nullptr) {
    ROS_DEBUG("[FrenetLatticePlanner::GenerateTrajectories] failed because lon and lat traj vec is nullptr");
    return;
  }

  auto end_condition_sampler = std::make_shared<EndConditionSampler>(init_s, init_d, ref_line, st_graph);
  FrenetLatticePlanner::GenerateLonTrajectories(maneuver_info, init_s, end_condition_sampler, ptr_lon_traj_vec);
  FrenetLatticePlanner::GenerateLatTrajectories(init_d, end_condition_sampler, ptr_lat_traj_vec);
  ROS_DEBUG("[FrenetLatticePlanner::GenerateTrajectories], "
            "the lon traj number is : %zu, the lat traj number is %zu",
            ptr_lon_traj_vec->size(), ptr_lat_traj_vec->size());
}

bool FrenetLatticePlanner::CombineTrajectories(const std::shared_ptr<ReferenceLine> &ptr_ref_line,
                                               const Polynomial &lon_traj_vec,
                                               const Polynomial &lat_traj_vec,
                                               std::shared_ptr<planning_msgs::Trajectory> ptr_combined_pub_traj) {
  return false;
}

void FrenetLatticePlanner::GenerateLatTrajectories(const std::array<double, 3> &init_d,
                                                   const std::shared_ptr<EndConditionSampler> &end_condition_sampler,
                                                   std::vector<std::shared_ptr<Polynomial>> *ptr_lat_traj_vec) {
  ptr_lat_traj_vec->clear();
  auto lat_end_condtions = end_condition_sampler->SampleLatEndCondition();
  FrenetLatticePlanner::GeneratePolynomialTrajectories(init_d, lat_end_condtions, 5, ptr_lat_traj_vec);
}

void FrenetLatticePlanner::GenerateLonTrajectories(const ManeuverInfo &maneuver_info,
                                                   const std::array<double, 3> &init_s,
                                                   const std::shared_ptr<EndConditionSampler> &end_condition_sampler,
                                                   std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) {
  if (ptr_lon_traj_vec == nullptr) {
    ROS_DEBUG("[FrenetLatticePlanner::GenerateLonTrajectories]. "
              "Failed to generate lon trajectories, because ptr_lon_traj_vec is nullptr");
    return;
  }
  ptr_lon_traj_vec->clear();
  FrenetLatticePlanner::GenerateCruisingLonTrajectories(PlanningConfig::Instance().target_speed(), init_s,
                                                        end_condition_sampler, ptr_lon_traj_vec);
  FrenetLatticePlanner::GenerateOvertakeAndFollowingLonTrajectories(init_s, end_condition_sampler, ptr_lon_traj_vec);
  if (maneuver_info.has_stop_point) {
    FrenetLatticePlanner::GenerateStoppingLonTrajectories(maneuver_info.maneuver_target.target_s,
                                                          init_s, end_condition_sampler,
                                                          ptr_lon_traj_vec);
  }
}

void FrenetLatticePlanner::GenerateCruisingLonTrajectories(double cruise_speed,
                                                           const std::array<double, 3> &init_s,
                                                           const std::shared_ptr<EndConditionSampler> &end_condition_sampler,
                                                           std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) {
  auto end_conditions = end_condition_sampler->SampleLonEndConditionForCruising(cruise_speed);
  FrenetLatticePlanner::GeneratePolynomialTrajectories(init_s, end_conditions, 4, ptr_lon_traj_vec);
}

void FrenetLatticePlanner::GenerateStoppingLonTrajectories(double stop_s,
                                                           const std::array<double, 3> &init_s,
                                                           const std::shared_ptr<EndConditionSampler> &end_condition_sampler,
                                                           std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) {
  auto end_conditions = end_condition_sampler->SampleLonEndConditionForStopping(stop_s);
  FrenetLatticePlanner::GeneratePolynomialTrajectories(init_s, end_conditions, 5, ptr_lon_traj_vec);
}

void FrenetLatticePlanner::GenerateOvertakeAndFollowingLonTrajectories(const std::array<double, 3> &init_s,
                                                                       const std::shared_ptr<EndConditionSampler> &end_condition_sampler,
                                                                       std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) {
  auto end_conditions = end_condition_sampler->SampleLonEndConditionWithSTGraph();
  FrenetLatticePlanner::GeneratePolynomialTrajectories(init_s, end_conditions, 5, ptr_lon_traj_vec);
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
  ROS_DEBUG("[FrenetLatticePlanner::GeneratePolynomialTrajectories], "
            "the end conditions vector's size: %zu", end_conditions.size());
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

void FrenetLatticePlanner::GetInitCondition(const std::shared_ptr<ReferenceLine> &ptr_ref_line,
                                            const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                            std::array<double, 3> *const init_s,
                                            std::array<double, 3> *const init_d) {
  ReferencePoint matched_ref_point;
  double matched_s;
  if (!ptr_ref_line->GetMatchedPoint(init_trajectory_point.path_point.x,
                                     init_trajectory_point.path_point.y,
                                     &matched_ref_point,
                                     &matched_s)) {
    ROS_FATAL("[FrenetLatticePlanner::Plan] failed because the GetMatchedPoint failed");
    return;
  }
  CoordinateTransformer::CartesianToFrenet(matched_s, matched_ref_point.heading(),
                                           matched_ref_point.x(),
                                           matched_ref_point.y(),
                                           matched_ref_point.kappa(),
                                           matched_ref_point.dkappa(),
                                           init_trajectory_point.path_point.x,
                                           init_trajectory_point.path_point.y,
                                           init_trajectory_point.vel,
                                           init_trajectory_point.acc,
                                           init_trajectory_point.path_point.theta,
                                           init_trajectory_point.path_point.kappa,
                                           init_s, init_d);
}

}