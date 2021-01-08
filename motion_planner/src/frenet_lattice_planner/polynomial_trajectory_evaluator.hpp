#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MOTION_PLANNER_FRENENT_LATTICE_PLANNER_POLYNOMIAL_TRAJECTORY_EVALUATOR_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MOTION_PLANNER_FRENENT_LATTICE_PLANNER_POLYNOMIAL_TRAJECTORY_EVALUATOR_HPP_

#include <array>
#include <memory>
#include <queue>
#include <ros/ros.h>
#include "curves/polynomial.hpp"
#include "obstacle_manager/st_graph.hpp"
#include "end_condition_sampler.hpp"
#include "planning_config.hpp"
#include "frenet_lattice_planner.hpp"

namespace planning {
class PolynomialTrajectoryEvaluator {
 public:
  typedef std::pair<std::shared_ptr<common::Polynomial>, std::shared_ptr<common::Polynomial>> TrajectoryPair;
  typedef std::pair<TrajectoryPair, double> TrajectoryCostPair;
  PolynomialTrajectoryEvaluator() = default;
  ~PolynomialTrajectoryEvaluator() = default;

  /**
   *
   * @param init_s
   * @param lon_trajectory_vec
   * @param lat_trajectory_vec
   * @param ref_line
   * @param ptr_st_graph
   */
  PolynomialTrajectoryEvaluator(const std::array<double, 3> &init_s,
                                const PlanningTarget &planning_target,
                                const std::vector<std::shared_ptr<common::Polynomial>> &lon_trajectory_vec,
                                const std::vector<std::shared_ptr<common::Polynomial>> &lat_trajectory_vec,
                                const ReferenceLine &ref_line,
                                std::shared_ptr<STGraph> ptr_st_graph,
                                common::ThreadPool *thread_pool);
  bool has_more_trajectory_pairs() const;
  size_t num_of_trajectory_pairs() const;
  double top_trajectory_pair_cost() const { return cost_queue_.top().second; }
  TrajectoryPair next_top_trajectory_pair() {
    ROS_ASSERT(has_more_trajectory_pairs());
    auto top = cost_queue_.top();
    cost_queue_.pop();
    return top.first;
  }
 private:
  double CentripetalAccelerationCost(
      const  std::shared_ptr<common::Polynomial>& lon_trajectory) const;
  double LatJerkCost(const std::shared_ptr<common::Polynomial> &lat_trajectory,
                     const std::shared_ptr<common::Polynomial> &lon_trajectory) const;
  static double LatOffsetCost(const std::shared_ptr<common::Polynomial> &lat_trajectory,
                              const std::shared_ptr<common::Polynomial> &lon_trajectory);
  static double LonJerkCost(const std::shared_ptr<common::Polynomial> &lon_trajectory);
  static double LonTargetCost(const std::shared_ptr<common::Polynomial> &lon_trajectory,
                              const PlanningTarget &planning_target);
  double LonCollisionCost(const std::shared_ptr<common::Polynomial> &lon_trajectory) const;

  static bool IsValidLongitudinalTrajectory(const common::Polynomial &lon_traj);

  static bool IsValidLateralTrajectory(const common::Polynomial& lat_traj);

  double Evaluate(const PlanningTarget &planning_target, const std::shared_ptr<common::Polynomial> &lon_traj,
                  const std::shared_ptr<common::Polynomial> &lat_traj);

  // comparator for priority queue
  struct Comparator : public std::binary_function<const TrajectoryCostPair &, const TrajectoryCostPair &, bool> {
    bool operator()(const TrajectoryCostPair &left, const TrajectoryCostPair &right) {
      return left.second > right.second;
    }
  };

 private:
  std::priority_queue<TrajectoryCostPair, std::vector<TrajectoryCostPair>, Comparator> cost_queue_;
  std::array<double, 3> init_s_{0.0, 0.0, 0.0};
  std::shared_ptr<STGraph> ptr_st_graph_;
  ReferenceLine ref_line_;

  std::vector<std::vector<std::pair<double, double>>> intervals_;

};
}
#endif