#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_
#include "math/math_utils.hpp"
#include <cppad/ipopt/solve.hpp>
#include <glog/logging.h>
//#include <planning_srvs/RouteResponse.h>
#include "reference_point.hpp"
#include <planning_msgs/WayPoint.h>

namespace planning {
class ReferenceLineSmoother {
 public:
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADVector;
  typedef CPPAD_TESTVECTOR(double) DVector;
  ReferenceLineSmoother() = default;
  ~ReferenceLineSmoother() = default;
  ReferenceLineSmoother(const double deviation_weight,
                        const double heading_weight,
                        const double distance_weight,
                        const double max_curvature);

//  bool SmoothReferenceLine(const planning_srvs::RouteResponse &route_response,
//                           std::vector<ReferencePoint> *const smoothed_ref_points);
  bool SmoothReferenceLine(const std::vector<planning_msgs::WayPoint> &waypoints,
                           std::vector<ReferencePoint> *const smoothed_ref_points);

//    bool GetSmoothReferenceLine(const ReferenceLine &raw_ref_line,
//                                ReferenceLine *smoothed_ref_line);
  void SetSmoothParams(const double deviation_weight,
                       const double heading_weight,
                       const double distance_weight,
                       const double max_curvature);
 private:
  bool SetUpConstraint();
  void SetUpOptions();
  void SetUpInitValue();
  bool TraceSmoothReferenceLine(
      const CppAD::ipopt::solve_result<DVector> &result,
      std::vector<ReferencePoint> *const smoothed_ref_line) const;

 private:
  std::vector<planning_msgs::WayPoint> way_points_;
  std::string options_;
  DVector x_l_;
  DVector x_u_;
  DVector g_l_;
  DVector g_u_;
  DVector xi_;
  double deviation_weight_ = 5.5;
  double heading_weight_ = 9.5;
  double distance_weight_ = 6.0;
  double max_curvature_ = 6.0;
};

}

#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_
