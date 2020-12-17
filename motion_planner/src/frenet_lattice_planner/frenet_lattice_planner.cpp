#include <frenet_lattice_planner/constraint_checker.hpp>
#include <planning_config.hpp>
#include "frenet_lattice_planner/frenet_lattice_planner.hpp"
#include "frenet_lattice_planner/polynomial_trajectory_evaluator.hpp"
#include "frenet_lattice_planner/end_condition_sampler.hpp"
#include "math/coordinate_transformer.hpp"
#include "frenet_lattice_planner/lattice_trajectory1d.hpp"
#include "collision_checker/collision_checker.hpp"

namespace planning {
using namespace common;

FrenetLatticePlanner::FrenetLatticePlanner(ThreadPool *thread_pool) : thread_pool_(thread_pool) {}

bool FrenetLatticePlanner::Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                   const std::vector<PlanningTarget> &planning_targets,
                                   planning_msgs::Trajectory &pub_trajectory,
                                   std::vector<planning_msgs::Trajectory> *valid_trajectories) {
  if (planning_targets.empty()) {
    ROS_FATAL("[FrenetLatticePlanner::Process]: ******No planning_targets provided*********88");
    return false;
  }
  ROS_INFO("[FrenetLatticePlanner::Process], the targets size: %zu", planning_targets.size());
  constexpr double kDefaultNonBestBehaviourCost = 20.0;
  size_t index = 0;
  size_t failed_ref_plan_num = 0;;
  std::vector<std::pair<planning_msgs::Trajectory, double>> optimal_trajectories;
  for (const auto &planning_target : planning_targets) {
    std::pair<planning_msgs::Trajectory, double> optimal_trajectory;
    auto result = PlanningOnRef(init_trajectory_point, planning_target, optimal_trajectory, valid_trajectories);
    if (!result) {
      ROS_FATAL("[FrenetLatticePlanner::Process], failed plan on reference line: %zu", index);
      failed_ref_plan_num++;
    }
    ++index;
    if (!planning_target.is_best_behaviour) {
      optimal_trajectory.second += kDefaultNonBestBehaviourCost;
    }
    optimal_trajectories.push_back(optimal_trajectory);
  }
  if (failed_ref_plan_num >= planning_targets.size()) {
    ROS_FATAL("[FrenetLatticePlanner::Process], the process is failed on every reference line");
    return false;
  }
  std::sort(optimal_trajectories.begin(), optimal_trajectories.end(),
            [](const std::pair<planning_msgs::Trajectory, double> &p0,
               const std::pair<planning_msgs::Trajectory, double> &p1) -> bool {
              return p0.second < p1.second;
            });

  pub_trajectory = std::move(optimal_trajectories.front().first);
  return true;
}

bool FrenetLatticePlanner::PlanningOnRef(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                         const PlanningTarget &planning_target,
                                         std::pair<planning_msgs::Trajectory, double> &optimal_trajectory,
                                         std::vector<planning_msgs::Trajectory> *valid_trajectories) const {
  ros::Time begin = ros::Time::now();
  std::shared_ptr<ReferenceLine> ref_line = planning_target.ref_lane;
  std::array<double, 3> init_s{};
  std::array<double, 3> init_d{};
  FrenetLatticePlanner::GetInitCondition(ref_line, init_trajectory_point, &init_s, &init_d);
  auto obstacle_vec = planning_target.obstacles;
  auto st_graph = std::make_shared<STGraph>(obstacle_vec, ref_line,
                                            init_s[0],
                                            init_s[0] + PlanningConfig::Instance().max_lookahead_distance(),
                                            0.0, PlanningConfig::Instance().max_lookahead_time(),
                                            init_d,
                                            PlanningConfig::Instance().max_lookahead_time(),
                                            PlanningConfig::Instance().delta_t());
  std::vector<std::shared_ptr<Polynomial>> lon_traj_vec;
  std::vector<std::shared_ptr<Polynomial>> lat_traj_vec;

  auto end_condition_sampler =
      std::make_shared<EndConditionSampler>(init_s, init_d, ref_line, planning_target.obstacles, st_graph);
  FrenetLatticePlanner::GenerateLonTrajectories(planning_target, init_s, end_condition_sampler, &lon_traj_vec);
  FrenetLatticePlanner::GenerateLatTrajectories(init_d, end_condition_sampler, &lat_traj_vec);
  ROS_INFO("[PlanningOnRef] : the lon end conditions size is %zu, the lat end_conditions size is %zu",
           lon_traj_vec.size(),
           lat_traj_vec.size());

  PolynomialTrajectoryEvaluator trajectory_evaluator = PolynomialTrajectoryEvaluator(init_s,
                                                                                     planning_target,
                                                                                     lon_traj_vec,
                                                                                     lat_traj_vec,
                                                                                     ref_line, st_graph,
                                                                                     thread_pool_);
  std::unordered_map<int, std::shared_ptr<Obstacle>> obstacle_map;
  for (const auto &obstacle : planning_target.obstacles) {
    obstacle_map.emplace(obstacle->Id(), obstacle);
  }
  CollisionChecker collision_checker = CollisionChecker(obstacle_map,
                                                        ref_line,
                                                        st_graph,
                                                        init_s[0],
                                                        init_d[0],
                                                        PlanningConfig::Instance().lon_safety_buffer(),
                                                        PlanningConfig::Instance().lat_safety_buffer(),
                                                        PlanningConfig::Instance().max_lookahead_time(),
                                                        PlanningConfig::Instance().delta_t(),
                                                        PlanningConfig::Instance().vehicle_params(),
                                                        thread_pool_);
  size_t collision_failure_count = 0;
  size_t combined_constraint_failure_count = 0;
  size_t lon_vel_failure_count = 0;
  size_t lon_acc_failure_count = 0;
  size_t lon_jerk_failure_count = 0;
  size_t curvature_failure_count = 0;
  size_t lat_acc_failure_count = 0;
  size_t lat_jerk_failure_count = 0;
  size_t num_lattice_traj = 0;
  ROS_INFO("[PlanningOnRef]: the trajectory pairs size is %zu", trajectory_evaluator.num_of_trajectory_pairs());
  if (!trajectory_evaluator.has_more_trajectory_pairs()) {
    ROS_FATAL("[PlanningOnRef]: Failed Reason: no Valid trajectory pairs");
  }
  while (trajectory_evaluator.has_more_trajectory_pairs()) {
    double trajectory_pair_cost = trajectory_evaluator.top_trajectory_pair_cost();
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();
    auto combined_trajectory = CombineTrajectories(ref_line, *trajectory_pair.first, *trajectory_pair.second,
                                                   init_trajectory_point.relative_time);
//    std::cout << " ===========the lat and lon traj is : " << std::endl;
//    for (double t = 0; t < PlanningConfig::Instance().max_lookahead_time(); t += 0.5) {
//      double s = trajectory_pair.first->Evaluate(0, t);
//      std::cout << "lon: t: " << t << ", s: " << s << ", s_dot: "
//                << trajectory_pair.first->Evaluate(1, t)
//                << ", s_ddot: " << trajectory_pair.first->Evaluate(2, t) << ", jerk: "
//                << trajectory_pair.first->Evaluate(3, t) << " lat d(s): " << trajectory_pair.second->Evaluate(0, s)
//                << ", d(s)': " << trajectory_pair.second->Evaluate(2, s) << ", d''(s): "
//                << trajectory_pair.second->Evaluate(2, s) << ", d'''(s): " << trajectory_pair.second->Evaluate(3, s)
//                << " the cost: " << trajectory_pair_cost << std::endl;
//    }

    auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);
    if (result != ConstraintChecker::Result::VALID) {
      ++combined_constraint_failure_count;
      switch (result) {
        case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND: {
          lon_vel_failure_count += 1;
          break;
        }
        case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND: {
          lon_acc_failure_count += 1;
          break;
        }
        case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND: {
          lon_jerk_failure_count += 1;
          break;
        }
        case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND: {
          curvature_failure_count += 1;
          break;
        }
        case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND: {
          lat_acc_failure_count += 1;
          break;
        }
        case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND: {
          lat_jerk_failure_count += 1;
          break;
        }
        case ConstraintChecker::Result::VALID:
        default: { break; }
      }
      continue;
    }
    if (collision_checker.IsCollision(combined_trajectory)) {
      ++collision_failure_count;
      continue;
    }

    num_lattice_traj += 1;
    optimal_trajectory.second = trajectory_pair_cost;
    optimal_trajectory.first = combined_trajectory;
    break;
    // set visualized trajectory vectors
//    if (valid_trajectories != nullptr) {
//      valid_trajectories->push_back(combined_trajectory);
//      if (num_lattice_traj == 1) {
//        optimal_trajectory.second = trajectory_pair_cost;
//        optimal_trajectory.first = combined_trajectory;
//      }
//    } else {
//      optimal_trajectory.second = trajectory_pair_cost;
//      optimal_trajectory.first = combined_trajectory;
//      break;
//    }
  }
  ROS_WARN(
      "[PlanningOnRef]: the lon_vel_failure_count:%zu,  lon_acc_failure_count: %zu,  lon_jerk_failure_count: %zu,  curvature_failure_count: %zu,"
      "lat_acc_failure_count: %zu, lat_jerk_failure_count: %zu",
      lon_vel_failure_count,
      lon_acc_failure_count,
      lon_jerk_failure_count,
      curvature_failure_count,
      lat_acc_failure_count,
      lat_jerk_failure_count);
  ros::Time end = ros::Time::now();
  ROS_INFO("[FrenetLatticePlanner::PlanningOnRef], the total time elapsed is %lf s", (end - begin).toSec());
#if 0
  std::cout << "---------the optimal trajectory is : ----------------" << std::endl;
  for (const auto &tp : optimal_trajectory.first.trajectory_points) {
    std::cout << " relative_time : " << tp.relative_time << ", s : " << tp.path_point.s << ", x : " << tp.path_point.x
              << " , y: " << tp.path_point.y << ", theta" << tp.path_point.theta << ", v : " << tp.vel << ", a: "
              << tp.acc << std::endl;

  }
#endif
  return num_lattice_traj > 0;
}

planning_msgs::Trajectory FrenetLatticePlanner::CombineTrajectories(const std::shared_ptr<ReferenceLine> &ptr_ref_line,
                                                                    const Polynomial &lon_traj,
                                                                    const Polynomial &lat_traj,
                                                                    double start_time) {
  double s0 = lon_traj.Evaluate(0, 0.0);
  double s_ref_max = ptr_ref_line->Length();
  double accumulated_s = 0.0;
  double last_s = -1.0 * std::numeric_limits<double>::epsilon();
  double t_param = 0.0;
  planning_msgs::PathPoint prev_path_point;
  planning_msgs::Trajectory combined_trajectory;
  while (t_param < PlanningConfig::Instance().max_lookahead_time()) {
    double s = lon_traj.Evaluate(0, t_param);
    if (last_s > 0.0) {
      s = std::max(last_s, s);
    }
    last_s = s;
    double s_dot = std::max(std::numeric_limits<double>::epsilon(), lon_traj.Evaluate(1, t_param));
    double s_dot_dot = lon_traj.Evaluate(2, t_param);
    if (s > s_ref_max) {
      break;
    }
    double relative_s = s - s0;
    double d = lat_traj.Evaluate(0, relative_s);
    double d_prime = lat_traj.Evaluate(1, relative_s);
    double d_prime_prime = lat_traj.Evaluate(2, relative_s);
    auto matched_re_point = ptr_ref_line->GetReferencePoint(s);
    double x = 0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double v = 0.0;
    double a = 0.0;
    const double rs = s;
    const double rx = matched_re_point.x();
    const double ry = matched_re_point.y();
    const double rtheta = matched_re_point.theta();
    const double rkappa = matched_re_point.kappa();
    const double rdkappa = matched_re_point.dkappa();
    std::array<double, 3> s_conditions = {rs, s_dot, s_dot_dot};
    std::array<double, 3> d_conditions = {d, d_prime, d_prime_prime};
    CoordinateTransformer::FrenetToCartesian(rs, rx, ry,
                                             rtheta, rkappa, rdkappa,
                                             s_conditions, d_conditions,
                                             &x, &y, &theta,
                                             &kappa, &v, &a);
    if (t_param > PlanningConfig::Instance().delta_t()) {
      double delta_x = x - prev_path_point.x;
      double delta_y = y - prev_path_point.y;
      accumulated_s += std::hypot(delta_x, delta_y);
    }
    planning_msgs::TrajectoryPoint tp;
    tp.path_point.x = x;
    tp.path_point.y = y;
    tp.path_point.theta = theta;
    tp.path_point.kappa = kappa;
    tp.path_point.s = accumulated_s;
    tp.vel = v;
    tp.acc = a;
    tp.relative_time = start_time + t_param;
    combined_trajectory.trajectory_points.push_back(tp);
    t_param += PlanningConfig::Instance().delta_t();
    prev_path_point = tp.path_point;
  }
#if 0
  std::cout << "====== the lon trajectory to evaluated is =========" << std::endl;
  double lon_end_s = lon_traj.Evaluate(0, lon_traj.ParamLength());
  for (double t = 0; t < lon_traj.ParamLength(); t += 0.2) {
    std::cout << "s : " << lon_traj.Evaluate(0, t) << ", dot_s: " << lon_traj.Evaluate(1, t) << ", ddot_s: "
              << lon_traj.Evaluate(2, t) << std::endl;
  }

  std::cout << "====== the lat trajectory to evaluated is =========" << std::endl;
  for (double s = 0; s < lon_end_s; s += 0.1) {
    std::cout << "l : " << lat_traj.Evaluate(0, s) << ", l_prime: " << lat_traj.Evaluate(1, s) << ", l_prime_prime: "
              << lat_traj.Evaluate(2, s) << std::endl;
  }

  std::cout << "========== the combined_trajectory is ==============" << std::endl;
  for (const auto &tp : combined_trajectory.trajectory_points) {
    std::cout << "relative_time: " << tp.relative_time << ", s: " << tp.path_point.s << ", x: " << tp.vel << ", a: "
              << tp.acc << std::endl;
  }
#endif

  return combined_trajectory;
}

void FrenetLatticePlanner::GenerateLatTrajectories(const std::array<double, 3> &init_d,
                                                   const std::shared_ptr<EndConditionSampler> &end_condition_sampler,
                                                   std::vector<std::shared_ptr<Polynomial>> *ptr_lat_traj_vec) {
  if (ptr_lat_traj_vec == nullptr) {
    return;
  }
  ptr_lat_traj_vec->clear();
  auto lat_end_conditions = end_condition_sampler->SampleLatEndCondition();
//  ROS_INFO("[FrenetLatticePlanner::GenerateLatTrajectories], the end conditions size : %zu", lat_end_conditions.size());

  FrenetLatticePlanner::GeneratePolynomialTrajectories(init_d, lat_end_conditions, 5, ptr_lat_traj_vec);

#if 0
  std::cout << "=============== the lat conditions is ============" << std::endl;
  for (const auto &lat_condition : lat_end_conditions) {
    std::cout << "d: " << lat_condition.first[0] << " d_prime: " << lat_condition.first[1] << ", d_prime_prime: "
              << lat_condition.first[2] << ", s: " << lat_condition.second << std::endl;
  }
  std::cout << "======= init d ======" << std::endl;
  std::cout << " init d : " << "l : " << init_d[0] << ", l_prime : " << init_d[1] << ", l_prime_prime: " << init_d[2]
            << std::endl;

  for (const auto &lat_traj : *ptr_lat_traj_vec) {
    std::cout << "the lat traj:" << std::endl;
    for (double s = 0; s < lat_traj->ParamLength(); s += 1) {
      std::cout << "s: " << s << ", l: " << lat_traj->Evaluate(0, s) << ", l_prime: " << lat_traj->Evaluate(1, s)
                << ", l_prime_prime: " << lat_traj->Evaluate(2, s) << std::endl;
    }
  }
#endif
}

void FrenetLatticePlanner::GenerateLonTrajectories(const PlanningTarget &planning_target,
                                                   const std::array<double, 3> &init_s,
                                                   const std::shared_ptr<EndConditionSampler> &end_condition_sampler,
                                                   std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) {
  if (ptr_lon_traj_vec == nullptr) {
    return;
  }
  ptr_lon_traj_vec->clear();
  auto ref_line = planning_target.ref_lane;
  auto matched_ref_point = ref_line->GetReferencePoint(init_s[0]);
//  std::cout << "=========== matched_ref_point: kappa: " << matched_ref_point.kappa() << std::endl;
  double cruise_speed = std::min(PlanningConfig::Instance().max_lon_velocity() * 0.9,
                                 PlanningConfig::Instance().max_lat_acc()
                                     / (std::fabs(matched_ref_point.kappa()) + 1e-6));
//  std::cout << " -==========================-- cruise speed is: " << cruise_speed << "m/s==============-" << std::endl;
  FrenetLatticePlanner::GenerateCruisingLonTrajectories(cruise_speed, init_s,
                                                        end_condition_sampler, ptr_lon_traj_vec);
  FrenetLatticePlanner::GenerateOvertakeAndFollowingLonTrajectories(init_s, end_condition_sampler, ptr_lon_traj_vec);
  if (planning_target.has_stop_point) {
    FrenetLatticePlanner::GenerateStoppingLonTrajectories(planning_target.stop_s,
                                                          init_s, end_condition_sampler,
                                                          ptr_lon_traj_vec);
  }
}

void FrenetLatticePlanner::GenerateCruisingLonTrajectories(double cruise_speed,
                                                           const std::array<double, 3> &init_s,
                                                           const std::shared_ptr<EndConditionSampler> &end_condition_sampler,
                                                           std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) {
  ros::Time begin = ros::Time::now();
  auto end_conditions = end_condition_sampler->SampleLonEndConditionForCruising(cruise_speed);
  FrenetLatticePlanner::GeneratePolynomialTrajectories(init_s, end_conditions, 4, ptr_lon_traj_vec);
  ros::Time end = ros::Time::now();
  ROS_INFO("[GenerateCruisingLonTrajectories], GeneratePolynomialTrajectories elapsed %lf s", (end - begin).toSec());
}

void FrenetLatticePlanner::GenerateStoppingLonTrajectories(double stop_s,
                                                           const std::array<double, 3> &init_s,
                                                           const std::shared_ptr<EndConditionSampler> &end_condition_sampler,
                                                           std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) {
  auto end_conditions = end_condition_sampler->SampleLonEndConditionForStopping(stop_s);
  ROS_INFO("[FrenetLatticePlanner::GenerateStoppingLonTrajectories], the end conditions size : %zu",
           end_conditions.size());
  ros::Time begin = ros::Time::now();
  FrenetLatticePlanner::GeneratePolynomialTrajectories(init_s, end_conditions, 5, ptr_lon_traj_vec);
  ros::Time end = ros::Time::now();
  ROS_INFO("[GenerateStoppingLonTrajectories], GeneratePolynomialTrajectories elapsed %lf s", (end - begin).toSec());
}

void FrenetLatticePlanner::GenerateOvertakeAndFollowingLonTrajectories(const std::array<double, 3> &init_s,
                                                                       const std::shared_ptr<EndConditionSampler> &end_condition_sampler,
                                                                       std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) {
  auto end_conditions = end_condition_sampler->SampleLonEndConditionWithSTGraph();
  ROS_INFO("[FrenetLatticePlanner::GenerateOvertakeAndFollowingLonTrajectories], the end conditions size : %zu",
           end_conditions.size());
  ros::Time begin = ros::Time::now();
  FrenetLatticePlanner::GeneratePolynomialTrajectories(init_s, end_conditions, 5, ptr_lon_traj_vec);
  ros::Time end = ros::Time::now();
  ROS_INFO("[GenerateOvertakeAndFollowingLonTrajectories], GeneratePolynomialTrajectories elapsed %lf s",
           (end - begin).toSec());

}

void FrenetLatticePlanner::GeneratePolynomialTrajectories(
    const std::array<double, 3> &init_condition,
    const std::vector<std::pair<std::array<double, 3>, double>> &end_conditions,
    size_t order, std::vector<std::shared_ptr<Polynomial>> *ptr_traj_vec) {

  if (ptr_traj_vec == nullptr) {
    ROS_FATAL("[FrenetLatticePlanner::GeneratePolynomialTrajectories], the pre_traj_vec is nullptr");
    return;
  }
  if (end_conditions.empty()) {
    ROS_INFO("[FrenetLatticePlanner::GeneratePolynomialTrajectories], the end conditions vector is empty");
    return;
  }
  ROS_INFO("[FrenetLatticePlanner::GeneratePolynomialTrajectories], "
           "the end conditions vector's size: %zu", end_conditions.size());
  ptr_traj_vec->reserve(ptr_traj_vec->size() + end_conditions.size());
  switch (order) {
    case 4: {
      for (const auto &end_condition : end_conditions) {
        auto ptr_trajectory = std::make_shared<LatticeTrajectory1d>(
            std::make_shared<QuarticPolynomial>(init_condition[0], init_condition[1], init_condition[2],
                                                end_condition.first[1], end_condition.first[2], end_condition.second));
        ptr_traj_vec->push_back(ptr_trajectory);
      }
      break;
    }
    case 5: {
      for (const auto &end_condition : end_conditions) {
        auto ptr_trajectory = std::make_shared<LatticeTrajectory1d>(
            std::make_shared<QuinticPolynomial>(init_condition, end_condition.first, end_condition.second));
        ptr_traj_vec->push_back(ptr_trajectory);
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
    ROS_FATAL("[FrenetLatticePlanner::GetInitCondition] failed because the GetMatchedPoint failed");
    return;
  }

  CoordinateTransformer::CartesianToFrenet(matched_s,
                                           matched_ref_point.x(),
                                           matched_ref_point.y(),
                                           matched_ref_point.theta(),
                                           matched_ref_point.kappa(),
                                           matched_ref_point.dkappa(),
                                           init_trajectory_point.path_point.x,
                                           init_trajectory_point.path_point.y,
                                           init_trajectory_point.vel,
                                           init_trajectory_point.acc,
                                           init_trajectory_point.path_point.theta,
                                           init_trajectory_point.path_point.kappa,
                                           init_s, init_d);
#if 0
  std::cout << "++++++++++GetInitConditions++++++++++" << std::endl;
  std::cout << "init_trajectory_point: x: " << init_trajectory_point.path_point.x << ", y: "
            << init_trajectory_point.path_point.y  << ", theta: " << init_trajectory_point.path_point.theta <<  ", v: " << init_trajectory_point.vel
            << ", a: " << init_trajectory_point.acc << ", relative_time: " << init_trajectory_point.relative_time
            << ", kappa: " << init_trajectory_point.path_point.kappa << ", dkappa: "
            << init_trajectory_point.path_point.dkappa
            << std::endl;
  std::cout << " ref_point: x: " << matched_ref_point.x() << ", y: " << matched_ref_point.y()
            << "theta: " << matched_ref_point.theta() << " kappa: " << matched_ref_point.kappa() << ", dkappa: "
            << matched_ref_point.dkappa()
            << ", matched_s: " << matched_s << std::endl;
  std::cout << "init_d: " << init_d->at(0)<< ", " << init_d->at(1) << ", " << init_d->at(2)<< std::endl;
#endif
}

void FrenetLatticePlanner::GenerateEmergencyStopTrajectory(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                                           planning_msgs::Trajectory &stop_trajectory) {
  const double kMaxTrajectoryTime = PlanningConfig::Instance().max_lookahead_time();
  const double kTimeGap = PlanningConfig::Instance().delta_t();
  stop_trajectory.trajectory_points.clear();
  auto num_traj_point = static_cast<int>(kMaxTrajectoryTime / kTimeGap);
  stop_trajectory.trajectory_points.resize(num_traj_point);
  const double max_decel = PlanningConfig::Instance().max_lon_acc();
  double stop_time = init_trajectory_point.vel / max_decel;
  double last_x = init_trajectory_point.path_point.x;
  double last_y = init_trajectory_point.path_point.y;
  double last_v = init_trajectory_point.vel;
  double last_a = max_decel;
  double last_theta = init_trajectory_point.path_point.theta;
  double last_s = init_trajectory_point.path_point.s;
  planning_msgs::TrajectoryPoint tp;
  tp = init_trajectory_point;
  tp.relative_time = 0.0;
  tp.acc = -max_decel;
  stop_trajectory.trajectory_points.push_back(tp);
  for (int i = 1; i < num_traj_point; ++i) {
    double t = i * kTimeGap + init_trajectory_point.relative_time;
    tp.relative_time = t;
    tp.vel = init_trajectory_point.vel - last_a * kTimeGap;
    tp.acc = t <= stop_time ? -max_decel : 0.0;
    tp.jerk = 0.0;
    double ds = (0.5 * last_a * kTimeGap + last_v) * kTimeGap;
    tp.path_point.x = last_x + std::cos(last_theta) * ds;
    tp.path_point.y = last_y + std::sin(last_theta) * ds;
    tp.path_point.theta = last_theta;
    tp.path_point.s = last_s + ds;
    tp.path_point.dkappa = 0.0;
    tp.path_point.kappa = 0.0;
    tp.steer_angle = init_trajectory_point.steer_angle;
    stop_trajectory.trajectory_points.push_back(tp);
    last_s = tp.path_point.s;
    last_x = tp.path_point.x;
    last_y = tp.path_point.y;
    last_theta = tp.path_point.theta;
    last_v = tp.vel;
    last_a = tp.acc;
  }

}

}