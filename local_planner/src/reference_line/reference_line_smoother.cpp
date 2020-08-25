#include "reference_line/reference_line_smoother.hpp"
#include "reference_line/reference_line_smooth_ipopt_interface.hpp"

namespace planning {

bool ReferenceLineSmoother::GetSmoothReferenceLine(
    const planning_srvs::RouteResponse &route_response,
    planning::ReferenceLine *smoothed_ref_line) {
  auto raw_ref_line = ReferenceLine(route_response);
  return this->GetSmoothReferenceLine(raw_ref_line, smoothed_ref_line);
}

bool ReferenceLineSmoother::GetSmoothReferenceLine(
    const std::vector<planning_msgs::WayPoint> &waypoints,
    planning::ReferenceLine *smoothed_ref_line) {
  auto raw_ref_line = ReferenceLine(waypoints);
  std::cout << "raw_ref_line: size: " << raw_ref_line.reference_points().size() << std::endl;
  return this->GetSmoothReferenceLine(raw_ref_line, smoothed_ref_line);
}

bool ReferenceLineSmoother::GetSmoothReferenceLine(
    const ReferenceLine &raw_ref_line,
    ReferenceLine *smoothed_ref_line) {
  if (smoothed_ref_line == nullptr) {
    ROS_FATAL("[ReferenceLineSmoother::GetSmoothReferenceLine],"
              " the smoothed_ref_line is nullptr");
    return false;
  }
  const size_t point_num = raw_ref_line.reference_points().size();
  if (point_num < 2) {
    ROS_FATAL("[ReferenceLineSmoother::GetSmoothReferenceLine], "
              "the ref points num is 1");
    return false;
  }
//  auto ref_line = raw_ref_line;
  raw_ref_line_ = raw_ref_line;
  if (point_num == 2 && !raw_ref_line_.RefineReferenceLine()) {
    ROS_FATAL("[ReferenceLineSmoother::GetSmoothReferenceLine], "
              "the point_num is 2, and failed to refine");
    return false;
  }
  ReferenceLineSmoothIpoptInterface smoother_interface(raw_ref_line_);
  SetUpOptions();
  SetUpInitValue();
  if (!this->SetUpConstraint()) {
    return false;
  }
  CppAD::ipopt::solve_result<DVector> solution;
  CppAD::ipopt::solve(options_, xi_, x_l_, x_u_, g_l_, g_u_,
                      smoother_interface, solution);
  if (solution.status != CppAD::ipopt::solve_result<DVector>::success) {
    ROS_FATAL("[GetSmoothReferenceLine] failed reason: %i",
              solution.status);
    return false;
  }
  *smoothed_ref_line = raw_ref_line;
  return this->TraceSmoothReferenceLine(solution, smoothed_ref_line);
}

bool ReferenceLineSmoother::SetUpConstraint() {
  const auto ref_points = raw_ref_line_.reference_points();
  size_t number_of_points = ref_points.size();
  size_t number_of_curvature_constraints = number_of_points - 2;
  x_l_.resize(number_of_points * 2);
  x_u_.resize(number_of_points * 2);
  g_l_.resize(0);
  g_u_.resize(0);
   g_l_.resize(number_of_curvature_constraints);
   g_u_.resize(number_of_curvature_constraints);
  std::cout << "number_of_curvature_constraints: " << number_of_curvature_constraints << std::endl;
  double half_vehicle_width = PlanningConfig::Instance().vehicle_params().width / 2.0;
  for (size_t i = 0; i < number_of_points; ++i) {
    size_t index = i * 2;
    double left_lane_width = raw_ref_line_.left_lane_width()[i];
    double right_lane_width = raw_ref_line_.right_lane_width()[i];
    double boundary_radius = (std::max(std::min(left_lane_width, right_lane_width) -
        half_vehicle_width, half_vehicle_width));
    x_l_[index] = ref_points[i].x() - boundary_radius;
    x_u_[index] = ref_points[i].x() + boundary_radius;
    x_l_[index + 1] = ref_points[i].y() - boundary_radius;
    x_u_[index + 1] = ref_points[i].y() + boundary_radius;
  }
  // for the last and begin point
  x_l_[0] = ref_points[0].x() - 0.1;
  x_u_[0] = ref_points[0].x() + 0.1;
  x_l_[1] = ref_points[0].y() - 0.1;
  x_u_[1] = ref_points[0].y() + 0.1;
  x_l_[2 * (number_of_points - 1)] = ref_points.back().x() - 0.2;
  x_u_[2 * (number_of_points - 1)] = ref_points.back().x() + 0.2;
  x_l_[2 * (number_of_points - 1) + 1] = ref_points.back().y() - 0.2;
  x_u_[2 * (number_of_points - 1) + 1] = ref_points.back().y() + 0.2;
  for(size_t i = 0; i < number_of_curvature_constraints; ++i){
    g_l_[i] = -PlanningConfig::Instance().reference_smoother_max_curvature();
    g_u_[i] = PlanningConfig::Instance().reference_smoother_max_curvature();
  }

  return true;
}

void ReferenceLineSmoother::SetUpOptions() {
  options_.clear();
  options_ += "Integer print_level  0\n";
  options_ += "Sparse  true        forward\n";
  options_ += "Sparse  true        reverse\n";
  options_ += "Numeric max_cpu_time  0.05\n";
  options_ += "Integer max_iter     20\n";
}

void ReferenceLineSmoother::SetUpInitValue() {
  size_t number_of_points = raw_ref_line_.reference_points().size();
  xi_.resize(number_of_points * 2);
  for (size_t i = 0; i < number_of_points; ++i) {
    size_t index = i * 2;
    xi_[index] = raw_ref_line_.reference_points()[i].x() ;
    xi_[index + 1] = raw_ref_line_.reference_points()[i].y() ;
  }
}

bool ReferenceLineSmoother::TraceSmoothReferenceLine(
    const CppAD::ipopt::solve_result<DVector> &result,
    planning::ReferenceLine *smoothed_ref_line) {

  if (result.status != CppAD::ipopt::solve_result<DVector>::success) {
    return false;
  }

  size_t number_of_points = raw_ref_line_.reference_points().size();

  if (result.x.size() != number_of_points * 2) {
    return false;
  }

  ReferencePoint reference_point;
  std::vector<ReferencePoint> ref_points;
  ref_points.reserve(number_of_points);

  for (size_t i = 0; i < number_of_points; ++i) {
    size_t index = i * 2;
    reference_point.set_xy(result.x[index], result.x[index + 1]);
    ref_points.push_back(reference_point);
  }

  return smoothed_ref_line->UpdateReferencePoints(ref_points);
}

}