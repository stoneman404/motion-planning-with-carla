#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_
#include "math/math_utils.hpp"
#include <cppad/ipopt/solve.hpp>
#include <glog/logging.h>
#include "reference_point.hpp"
#include <planning_msgs/WayPoint.h>

namespace planning {
class ReferenceLineSmoother {
 public:
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADVector;
  typedef CPPAD_TESTVECTOR(double) DVector;
  ReferenceLineSmoother() = default;
  ~ReferenceLineSmoother() = default;
  ReferenceLineSmoother(double deviation_weight,
                        double heading_weight,
                        double distance_weight,
                        double max_curvature);

//  bool SmoothReferenceLine(const planning_srvs::RouteResponse &route_response,
//                           std::vector<ReferencePoint> *const smoothed_ref_points);
  bool SmoothReferenceLine(const std::vector<ReferencePoint> &raw_points,
                           std::vector<ReferencePoint> *smoothed_ref_points);

//    bool GetSmoothReferenceLine(const ReferenceLine &raw_ref_line,
//                                ReferenceLine *smoothed_ref_line);
  void SetSmoothParams(double deviation_weight,
                       double distance_weight,
                       double heading_weight,
                       double slack_weight,
                       double max_curvature);
 private:
  bool SetUpConstraint();
  void SetUpOptions();
  void SetUpInitValue();
  bool TraceSmoothReferenceLine(
      const CppAD::ipopt::solve_result<DVector> &result,
      std::vector<ReferencePoint> *smoothed_ref_line) const;

 private:
  std::vector<ReferencePoint> ref_points_;
  std::string options_;
  DVector x_l_;
  DVector x_u_;
  DVector g_l_;
  DVector g_u_;
  DVector xi_;
  double deviation_weight_ = 7.5;
  double heading_weight_ = 60.0;
  double distance_weight_ = 1.0;
  double max_curvature_ = 5.0;
  double slack_weight_ = 5.0;
  size_t num_of_points_{};
  size_t slack_variable_start_index_{};
  size_t num_of_slack_variable_{};
  size_t slack_variable_end_index_{};
  size_t num_curvature_constraint_{};
  size_t curvature_constraint_start_index_{};
  size_t curvature_constraint_end_index_{};
  size_t num_of_variables_{};
  size_t num_of_slack_constr_{};
  size_t num_of_constraint_{};
  size_t slack_constraint_start_index_{};
  size_t slack_constraint_end_index_{};
};

}

#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_
