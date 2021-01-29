#include "frenet_lattice_planner/polynomial_trajectory_evaluator.hpp"
#include <utility>
#include <planning_config.hpp>
#include "frenet_lattice_planner/constraint_checker.hpp"
#include "frenet_lattice_planner.hpp"

namespace planning {
PolynomialTrajectoryEvaluator::PolynomialTrajectoryEvaluator(const std::array<double, 3> &init_s,
                                                             const PlanningTarget &planning_target,
                                                             const std::vector<std::shared_ptr<common::Polynomial>> &lon_trajectory_vec,
                                                             const std::vector<std::shared_ptr<common::Polynomial>> &lat_trajectory_vec,
                                                             const ReferenceLine &ref_line,
                                                             std::shared_ptr<STGraph> ptr_st_graph,
                                                             common::ThreadPool *thread_pool)
    : init_s_(init_s), ptr_st_graph_(std::move(ptr_st_graph)),
      ref_line_(ref_line) {
  double start_time = 0.0;
  double end_time = PlanningConfig::Instance().max_lookahead_time();
  intervals_ = ptr_st_graph_->GetPathBlockingIntervals(start_time, end_time, PlanningConfig::Instance().delta_t());
  double stop_point = std::numeric_limits<double>::max();
  if (planning_target.has_stop_point) {
    stop_point = planning_target.stop_s;
  }
  auto begin = ros::Time::now();
  if (thread_pool != nullptr) {
    std::vector<std::future<TrajectoryCostPair>> futures;
    for (const auto &lon_traj : lon_trajectory_vec) {
      double lon_end_s = lon_traj->Evaluate(0, end_time);
      if (init_s[0] < stop_point && lon_end_s +
          PlanningConfig::Instance().lon_safety_buffer() > stop_point) {
        continue;
      }
      if (!IsValidLongitudinalTrajectory(*lon_traj)) {
        continue;
      }
      for (const auto &lat_traj : lat_trajectory_vec) {
        auto lambda = [&lon_traj, &lat_traj, &planning_target, this]() -> TrajectoryCostPair {
          double cost = Evaluate(planning_target, lon_traj, lat_traj);
          return TrajectoryCostPair(TrajectoryPair(lon_traj, lat_traj), cost);
        };
        futures.push_back(thread_pool->PushTask(lambda));
      }
    }
    for (auto &task : futures) {
      cost_queue_.emplace(task.get());
    }
  } else {
    for (const auto &lon_traj : lon_trajectory_vec) {
      double lon_end_s = lon_traj->Evaluate(0, end_time);
      if (init_s[0] < stop_point && lon_end_s +
          PlanningConfig::Instance().lon_safety_buffer() > stop_point) {
        continue;
      }
      if (!IsValidLongitudinalTrajectory(*lon_traj)) {
        continue;
      }


      for (const auto &lat_traj : lat_trajectory_vec) {
//        if (!IsValidLateralTrajectory(*lon_traj, *lat_traj)) {
//          continue;
//        }
        double cost = Evaluate(planning_target, lon_traj, lat_traj);
        cost_queue_.emplace(TrajectoryPair(lon_traj, lat_traj), cost);
      }
    }
  }

  auto end = ros::Time::now();
  ROS_WARN("[PolynomialTrajectoryEvaluator], the time elapsed by PolynomialTrajectoryEvaluator is %lf s",
           (end - begin).toSec());
  ROS_INFO("[PolynomialTrajectoryEvaluator], the numeber of trajectory pairs: %zu", cost_queue_.size());
}

bool PolynomialTrajectoryEvaluator::IsValidLongitudinalTrajectory(const common::Polynomial &lon_traj) {

  double t = 0.0;
  while (t < lon_traj.ParamLength()) {
    double v = lon_traj.Evaluate(1, t);
    if (!ConstraintChecker::WithInRange(v, PlanningConfig::Instance().min_lon_velocity(),
                                        PlanningConfig::Instance().max_lon_velocity())) {

//      ROS_FATAL("[PolynomialTrajectoryEvaluator], the lon_traj is not valid, because **LON_VEL** exceeds the  vel range. vel: %lf", v);
      return false;
    }
    double a = lon_traj.Evaluate(2, t);
    if (!ConstraintChecker::WithInRange(a,
                                        PlanningConfig::Instance().min_lon_acc(),
                                        PlanningConfig::Instance().max_lon_acc())) {
//      ROS_FATAL("[PolynomialTrajectoryEvaluator], the lon_traj is not valid, because **LON_ACC** exceeds the  acc range. a: %lf", a);
      return false;
    }
    double j = lon_traj.Evaluate(3, t);
    if (!ConstraintChecker::WithInRange(j,
                                        PlanningConfig::Instance().min_lon_jerk(),
                                        PlanningConfig::Instance().max_lon_jerk())) {
//      ROS_FATAL("[PolynomialTrajectoryEvaluator], the lon_traj is not valid, because **LON_JERK** exceeds the  jerk range. jerk: %lf", j);
      return false;
    }
    t += PlanningConfig::Instance().delta_t();
  }
  return true;
}

bool PolynomialTrajectoryEvaluator::IsValidLateralTrajectory(const common::Polynomial &lon_traj,
                                                             const common::Polynomial &lat_traj) {
  double t = 0.0;
  while (t < lon_traj.ParamLength()) {
    double s = lon_traj.Evaluate(0, t);
    double l = lat_traj.Evaluate(0, s);
    double dsdt = lon_traj.Evaluate(1, t);
    double dsddt = lon_traj.Evaluate(2, t);
    double dlds = lat_traj.Evaluate(1, s);
    double dldds = lat_traj.Evaluate(2, s);
    double dldt = dlds * dsdt;
    double dlddt = dldds * dsdt * dsdt + dlds * dsddt;
    if (!ConstraintChecker::WithInRange(l, -4.0, 4.0)) {
      return false;
    }
//    if (!ConstraintChecker::WithInRange(dlddt,
//                                        PlanningConfig::Instance().min_lat_acc(),
//                                        PlanningConfig::Instance().max_lat_acc())) {
//      return false;
//    }
  }
  return true;
}

double PolynomialTrajectoryEvaluator::Evaluate(const PlanningTarget &planning_target,
                                               const std::shared_ptr<common::Polynomial> &lon_traj,
                                               const std::shared_ptr<common::Polynomial> &lat_traj) {
  double lon_target_cost = PolynomialTrajectoryEvaluator::LonTargetCost(lon_traj, planning_target);
  double lon_jerk_cost = PolynomialTrajectoryEvaluator::LonJerkCost(lon_traj);
  double lon_collision_cost = this->LonCollisionCost(lon_traj);
  double lat_offset_cost = PolynomialTrajectoryEvaluator::LatOffsetCost(lat_traj, lon_traj);
  double lat_jerk_cost = this->LatJerkCost(lat_traj, lon_traj);
  double centripental_cost = this->CentripetalAccelerationCost(lon_traj);
//  std::cout << " lon_target_cost: " << lon_target_cost << ",lon_jerk_cost: " << lon_jerk_cost
//            << ", lon_collision_cost: " << lon_collision_cost
//            << ", lat_offset_cost: " << lat_offset_cost << ", lat_jerk_cost: " << lat_jerk_cost
//            << ", centripental_cost: " << centripental_cost << std::endl;
  return lon_collision_cost * PlanningConfig::Instance().lattice_weight_collision() +
      lon_jerk_cost * PlanningConfig::Instance().lattice_weight_lon_jerk() +
      lon_target_cost * PlanningConfig::Instance().lattice_weight_lon_target() +
      lat_jerk_cost * PlanningConfig::Instance().lattice_weight_lat_jerk() +
      lat_offset_cost * PlanningConfig::Instance().lattice_weight_lat_offset() +
      centripental_cost * PlanningConfig::Instance().lattice_weight_centripetal_acc();
}

size_t PolynomialTrajectoryEvaluator::num_of_trajectory_pairs() const {
  return cost_queue_.size();
}
bool PolynomialTrajectoryEvaluator::has_more_trajectory_pairs() const {
  return !cost_queue_.empty();
}

double PolynomialTrajectoryEvaluator::LatJerkCost(const std::shared_ptr<common::Polynomial> &lat_trajectory,
                                                  const std::shared_ptr<common::Polynomial> &lon_trajectory) const {

  double max_cost = 0.0;
  for (double t = 0.0; t < PlanningConfig::Instance().max_lookahead_time();
       t += PlanningConfig::Instance().delta_t()) {
    double s = lon_trajectory->Evaluate(0, t);
    double s_dot = lon_trajectory->Evaluate(1, t);
    double s_dotdot = lon_trajectory->Evaluate(2, t);

    double relative_s = s - init_s_[0];
    double l_prime = lat_trajectory->Evaluate(1, relative_s);
    double l_primeprime = lat_trajectory->Evaluate(2, relative_s);
    double cost = l_primeprime * s_dot * s_dot + l_prime * s_dotdot;
    max_cost = std::max(max_cost, std::fabs(cost));
  }
  return max_cost;
}

double PolynomialTrajectoryEvaluator::LatOffsetCost(const std::shared_ptr<common::Polynomial> &lat_trajectory,
                                                    const std::shared_ptr<common::Polynomial> &lon_trajectory) {
  const double param_length = lon_trajectory->ParamLength();
  double evaluation_horizon = std::min(PlanningConfig::Instance().max_lookahead_distance(),
                                       lon_trajectory->Evaluate(0, param_length));
  std::vector<double> s_values;
  for (double s = 0.0; s < evaluation_horizon; s += 0.1) {
    s_values.emplace_back(s);
  }
  double lat_offset_start = lat_trajectory->Evaluate(0, 0.0);
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (const auto &s : s_values) {
    double lat_offset = lat_trajectory->Evaluate(0, s);
    double cost = lat_offset / 3.0;
    if (lat_offset * lat_offset_start < 0.0) {
      cost_sqr_sum += cost * cost * PlanningConfig::Instance().lattice_weight_opposite_side_offset();
      cost_abs_sum += std::fabs(cost) * PlanningConfig::Instance().lattice_weight_opposite_side_offset();
    } else {
      cost_sqr_sum += cost * cost * PlanningConfig::Instance().lattice_weight_same_side_offset();
      cost_abs_sum += std::fabs(cost) * PlanningConfig::Instance().lattice_weight_same_side_offset();
    }
  }
  return cost_sqr_sum / (cost_abs_sum + 1e-5);
}

double PolynomialTrajectoryEvaluator::LonJerkCost(const std::shared_ptr<common::Polynomial> &lon_trajectory) {
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (double t = 0.0; t < PlanningConfig::Instance().max_lookahead_time();
       t += PlanningConfig::Instance().delta_t()) {
    double jerk = lon_trajectory->Evaluate(3, t);
    double cost = jerk / PlanningConfig::Instance().max_lon_jerk();
    cost_sqr_sum += cost * cost;
    cost_abs_sum += std::fabs(cost);
  }
  return cost_sqr_sum / (cost_abs_sum + 1.0e-5);
}

double PolynomialTrajectoryEvaluator::LonTargetCost(const std::shared_ptr<common::Polynomial> &lon_trajectory,
                                                    const PlanningTarget &planning_target) {

  double t_max = lon_trajectory->ParamLength();
  double dist_s = lon_trajectory->Evaluate(0, t_max) - lon_trajectory->Evaluate(0, 0.0);
//  std::cout << " ............dist_s: " << dist_s << std::endl;
  double speed_cost_sqr_sum = 0.0;
  double speed_cost_weight_sum = 0.0;
//  ROS_INFO("LonTargetCost: the desired vel is %f", planning_target.desired_vel);
  double target_speed = planning_target.has_stop_point ? 0.0 : planning_target.desired_vel;
  for (double t = 0; t <= t_max; t += PlanningConfig::Instance().delta_t()) {
    double cost = target_speed - lon_trajectory->Evaluate(1, t);
//    std::cout << " ============cost:=======      " << cost << ", lon_trajectory->Evaluate(1, t): "
//              << lon_trajectory->Evaluate(1, t) << ",  target_speed: " << target_speed << std::endl;

    speed_cost_sqr_sum += t * t * std::fabs(cost);
    speed_cost_weight_sum += t * t;
  }
  double speed_cost = speed_cost_sqr_sum / (speed_cost_weight_sum + 1e-5);

  double dist_travelled_cost = 1.0 / (1.0 + std::fabs(dist_s));
//  std::cout << " speed cost: " << speed_cost << "dist_travleed_cost: " << dist_travelled_cost << std::endl;
  return speed_cost * PlanningConfig::Instance().lattice_weight_target_speed() +
      dist_travelled_cost * PlanningConfig::Instance().lattice_weight_dist_travelled();
}

double PolynomialTrajectoryEvaluator::LonCollisionCost(const std::shared_ptr<common::Polynomial> &lon_trajectory) const {
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (size_t i = 0; i < intervals_.size(); ++i) {
    const auto &pt_interval = intervals_[i];
    if (pt_interval.empty()) {
      continue;
    }
    double t = static_cast<double>(i) * PlanningConfig::Instance().delta_t();
    double traj_s = lon_trajectory->Evaluate(0, t);
    double sigma = 2.0;
    for (const auto &m : pt_interval) {
      double dist = 0.0;
      // yeild
      if (traj_s < m.first - PlanningConfig::Instance().lon_safety_buffer()) {
        dist = m.first - PlanningConfig::Instance().lon_safety_buffer() - traj_s;
        // overtake
      } else if (traj_s > m.second + PlanningConfig::Instance().lon_safety_buffer()) {
        dist = traj_s - m.second - PlanningConfig::Instance().lon_safety_buffer();
      }
      double cost = std::exp(-dist * dist / (2.0 * sigma * sigma));

      cost_sqr_sum += cost * cost;
      cost_abs_sum += cost;
    }
  }
  return cost_sqr_sum / (cost_abs_sum + 1e-5);
}

double PolynomialTrajectoryEvaluator::CentripetalAccelerationCost(const std::shared_ptr<common::Polynomial> &lon_trajectory) const {
  // Assumes the vehicle is not obviously deviate from the reference line.
  double centripetal_acc_sum = 0.0;
  double centripetal_acc_sqr_sum = 0.0;
  for (double t = 0.0; t < PlanningConfig::Instance().max_lookahead_time();
       t += PlanningConfig::Instance().delta_t()) {
    double s = lon_trajectory->Evaluate(0, t);
    double v = lon_trajectory->Evaluate(1, t);
    auto ref_point = ref_line_.GetReferencePoint(s);
    double centripetal_acc = v * v * ref_point.kappa();
    centripetal_acc_sum += std::fabs(centripetal_acc);
    centripetal_acc_sqr_sum += centripetal_acc * centripetal_acc;
  }

  return centripetal_acc_sqr_sum /
      (centripetal_acc_sum + 1e-5);
}

}