#include "motion_planner/frenet_lattice_planner/polynomial_trajectory_evaluator.hpp"

#include <utility>

namespace planning {

PolynomialTrajectoryEvaluator::PolynomialTrajectoryEvaluator(const std::array<double, 3> &init_s,
                                                             const std::vector<std::shared_ptr<Polynomial>> &lon_trajectory_vec,
                                                             const std::vector<std::shared_ptr<Polynomial>> &lat_trajectory_vec,
                                                             std::shared_ptr<ReferenceLine> ptr_ref_line,
                                                             std::shared_ptr<STGraph> ptr_st_graph)
    : init_s_(init_s), ptr_st_graph_(std::move(ptr_st_graph)),
      ptr_ref_line_(std::move(ptr_ref_line)) {}

}