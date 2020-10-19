#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MOTION_PLANNER_FRENENT_LATTICE_PLANNER_POLYNOMIAL_TRAJECTORY_EVALUATOR_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MOTION_PLANNER_FRENENT_LATTICE_PLANNER_POLYNOMIAL_TRAJECTORY_EVALUATOR_HPP_

#include <array>
#include <memory>
#include <queue>
#include <ros/ros.h>
#include <planning_context.hpp>

#include "polynomial.hpp"
#include "collision_checker/st_graph.hpp"

namespace planning {
class PolynomialTrajectoryEvaluator {
  typedef std::pair<std::shared_ptr<Polynomial>, std::shared_ptr<Polynomial>> TrajectoryPair;
  typedef std::pair<TrajectoryPair, double> TrajectoryCostPair;

 public:
  PolynomialTrajectoryEvaluator() = default;
  ~PolynomialTrajectoryEvaluator() = default;

  /**
   *
   * @param init_s
   * @param lon_trajectory_vec
   * @param lat_trajectory_vec
   * @param ptr_ref_line
   * @param ptr_st_graph
   */
  PolynomialTrajectoryEvaluator(const std::array<double, 3> &init_s,
                                const ManeuverInfo &maneuver_info,
                                const std::vector<std::shared_ptr<Polynomial>> &lon_trajectory_vec,
                                const std::vector<std::shared_ptr<Polynomial>> &lat_trajectory_vec,
                                std::shared_ptr<ReferenceLine> ptr_ref_line,
                                std::shared_ptr<STGraph> ptr_st_graph);
  bool TrajectoryPairsIsEmpty() const;
  size_t TrajectoryPairSize() const;
 private:

  double LatJerkCost(const std::shared_ptr<Polynomial> &lat_trajectory,
                     const std::shared_ptr<Polynomial> &lon_trajectory) const;
  static double LatOffsetCost(const std::shared_ptr<Polynomial> &lat_trajectory,
                              const std::shared_ptr<Polynomial> &lon_trajectory);
  double LonJerkCost(const std::shared_ptr<Polynomial> &lon_trajectory) const;
  double LonTargetCost(const std::shared_ptr<Polynomial> &lon_trajectory,
                       const ManeuverInfo &maneuver_info) const;
  double LonCollisionCost(const std::shared_ptr<Polynomial> &lon_trajectory) const;

  static bool IsValidLongitudinalTrajectory(const Polynomial &lon_traj);
  static bool WithInRange(double value, double lower, double upper, double eps = 1e-4);
  double Evaluate(const ManeuverInfo &maneuver_info, const std::shared_ptr<Polynomial> &lon_traj,
                  const std::shared_ptr<Polynomial> &lat_traj);

  // comparator for priority queue
  struct Comparator : public std::binary_function<const TrajectoryCostPair &, const TrajectoryCostPair &, bool> {
    bool operator()(const TrajectoryCostPair &left, const TrajectoryCostPair &right) {
      return left.second > right.second;
    }
  };

 private:
  std::priority_queue<TrajectoryCostPair, std::vector<TrajectoryCostPair>, Comparator> cost_queue_;
  std::array<double, 3> init_s_{0.0, 0.0, 0.0};
  std::shared_ptr<ReferenceLine> ptr_ref_line_;
  std::shared_ptr<STGraph> ptr_st_graph_;
  std::vector<std::vector<std::pair<double, double>>> intervals_;

};
}
#endif