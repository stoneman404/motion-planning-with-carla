#include "motion_planner/frenet_lattice_planner/polynomial_trajectory_evaluator.hpp"
#include <utility>
#include "motion_planner/frenet_lattice_planner/constraint_checker.hpp"

namespace planning {
PolynomialTrajectoryEvaluator::PolynomialTrajectoryEvaluator(const std::array<double, 3> &init_s,
                                                             const ManeuverInfo &maneuver_info,
                                                             const std::vector<std::shared_ptr<Polynomial>> &lon_trajectory_vec,
                                                             const std::vector<std::shared_ptr<Polynomial>> &lat_trajectory_vec,
                                                             std::shared_ptr<ReferenceLine> ptr_ref_line,
                                                             std::shared_ptr<STGraph> ptr_st_graph)
    : init_s_(init_s), ptr_st_graph_(std::move(ptr_st_graph)),
      ptr_ref_line_(std::move(ptr_ref_line)) {
  double start_time = 0.0;
  double end_time = PlanningConfig::Instance().max_lookahead_time();
  intervals_ = ptr_st_graph_->GetPathBlockingIntervals(start_time, end_time, PlanningConfig::Instance().delta_t());
  double stop_point = std::numeric_limits<double>::max();
  if (maneuver_info.has_stop_point) {
    stop_point = maneuver_info.maneuver_target.target_s;
  }
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
      double cost = Evaluate(maneuver_info, lon_traj, lat_traj);
      cost_queue_.emplace(TrajectoryPair(lon_traj, lat_traj), cost);
    }
  }
  ROS_DEBUG("[PolynomialTrajectoryEvaluator], the numeber of trajectory pairs: %zu", cost_queue_.size());
}

bool PolynomialTrajectoryEvaluator::IsValidLongitudinalTrajectory(const Polynomial &lon_traj) {

  double t = 0.0;
  while (t < lon_traj.ParamLength()) {
    double v = lon_traj.Evaluate(1, t);
    if (!ConstraintChecker::WithInRange(v, PlanningConfig::Instance().min_lon_velocity(),
                                        PlanningConfig::Instance().max_lon_velocity())) {
      return false;
    }
    double a = lon_traj.Evaluate(2, t);
    if (!ConstraintChecker::WithInRange(a,
                                        PlanningConfig::Instance().min_lon_acc(),
                                        PlanningConfig::Instance().max_lon_acc())) {
      return false;
    }
    double j = lon_traj.Evaluate(3, t);
    if (!ConstraintChecker::WithInRange(j,
                                        PlanningConfig::Instance().min_lon_jerk(),
                                        PlanningConfig::Instance().max_lon_jerk())) {
      return false;
    }
    t += PlanningConfig::Instance().delta_t();
  }
  return true;
}

double PolynomialTrajectoryEvaluator::Evaluate(const ManeuverInfo &maneuver_info,
                                               const std::shared_ptr<Polynomial> &lon_traj,
                                               const std::shared_ptr<Polynomial> &lat_traj) {
  double lon_target_cost = PolynomialTrajectoryEvaluator::LonTargetCost(lon_traj, maneuver_info);
  double lon_jerk_cost = PolynomialTrajectoryEvaluator::LonJerkCost(lon_traj);
  double lon_collision_cost = this->LonCollisionCost(lon_traj);
  double lat_offset_cost = PolynomialTrajectoryEvaluator::LatOffsetCost(lat_traj, lon_traj);
  double lat_jerk_cost = this->LatJerkCost(lat_traj, lon_traj);
  return lon_collision_cost * PlanningConfig::Instance().lattice_weight_collision() +
      lon_jerk_cost * PlanningConfig::Instance().lattice_weight_lon_jerk() +
      lon_target_cost * PlanningConfig::Instance().lattice_weight_lon_target() +
      lat_jerk_cost * PlanningConfig::Instance().lattice_weight_lat_jerk() +
      lat_offset_cost * PlanningConfig::Instance().lattice_weight_lat_offset();
}

size_t PolynomialTrajectoryEvaluator::num_of_trajectory_pairs() const {
  return cost_queue_.size();
}
bool PolynomialTrajectoryEvaluator::has_more_trajectory_pairs() const {
  return !cost_queue_.empty();
}

double PolynomialTrajectoryEvaluator::LatJerkCost(const std::shared_ptr<Polynomial> &lat_trajectory,
                                                  const std::shared_ptr<Polynomial> &lon_trajectory) const {

  double cost = 0.0;
  for (double t = 0.0; t < PlanningConfig::Instance().max_lookahead_time();
       t += PlanningConfig::Instance().delta_t()) {
    double s = lon_trajectory->Evaluate(0, t);
    double s_d = lon_trajectory->Evaluate(1, t);
    double s_dd = lon_trajectory->Evaluate(2, t);
    double relative_s = s - init_s_[0];
    double l_prime = lat_trajectory->Evaluate(1, relative_s);
    double l_prime_prime = lat_trajectory->Evaluate(2, relative_s);
    cost += std::pow(l_prime_prime * s_d * s_d + l_prime * s_dd, 2);
  }
  return cost;
}

double PolynomialTrajectoryEvaluator::LatOffsetCost(const std::shared_ptr<Polynomial> &lat_trajectory,
                                                    const std::shared_ptr<Polynomial> &lon_trajectory) {
  const double param_length = lon_trajectory->ParamLength();
  double evaluation_horizon = std::min(PlanningConfig::Instance().max_lookback_distance(),
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
    double cost = lat_offset / 100;
    if (lat_offset * lat_offset_start < 0.0) {
      cost_sqr_sum += cost * cost * PlanningConfig::Instance().lattice_weight_opposite_side_offset();
      cost_abs_sum += std::fabs(cost) * PlanningConfig::Instance().lattice_weight_opposite_side_offset();
    } else {
      cost_sqr_sum += cost * cost * PlanningConfig::Instance().lattice_weight_same_side_offset();
      cost_abs_sum += std::fabs(cost) * PlanningConfig::Instance().lattice_weight_same_side_offset();
    }
  }
  return cost_sqr_sum / (cost_abs_sum + 1e-3);
}

double PolynomialTrajectoryEvaluator::LonJerkCost(const std::shared_ptr<Polynomial> &lon_trajectory) {
  double cost = 0.0;
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (double t = 0.0; t < PlanningConfig::Instance().max_lookahead_time();
       t += PlanningConfig::Instance().delta_t()) {
    double jerk = lon_trajectory->Evaluate(3, t);
    cost_sqr_sum += std::pow(jerk / PlanningConfig::Instance().max_lon_jerk(), 2);
    cost_abs_sum += std::fabs(jerk / PlanningConfig::Instance().max_lon_jerk());
  }
  return cost_sqr_sum / (cost_abs_sum + 1e-3);
}

double PolynomialTrajectoryEvaluator::LonTargetCost(const std::shared_ptr<Polynomial> &lon_trajectory,
                                                    const ManeuverInfo &maneuver_info) {

  double t_max = lon_trajectory->ParamLength();
  double dist_s = lon_trajectory->Evaluate(0, t_max) - lon_trajectory->Evaluate(0, 0.0);
  double speed_cost_sqr_sum = 0.0;
  double speed_cost_weight_sum = 0.0;
  double target_speed = maneuver_info.has_stop_point ? 0.0 : maneuver_info.maneuver_target.target_speed;
  for (double t = 0; t <= t_max; t += PlanningConfig::Instance().delta_t()) {
    double cost = target_speed - lon_trajectory->Evaluate(1, t);
    speed_cost_sqr_sum += t * t * std::fabs(cost);
    speed_cost_weight_sum += t * t;
  }
  double speed_cost = speed_cost_sqr_sum / (speed_cost_weight_sum + 1e-3);
  double dist_travelled_cost = 1.0 / (1.0 + dist_s);
  return speed_cost * PlanningConfig::Instance().lattice_weight_target_speed() +
      dist_travelled_cost * PlanningConfig::Instance().lattice_weight_dist_travelled();
}

double PolynomialTrajectoryEvaluator::LonCollisionCost(const std::shared_ptr<Polynomial> &lon_trajectory) const {
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
  return cost_sqr_sum / (cost_abs_sum + 1e-3);
}

}