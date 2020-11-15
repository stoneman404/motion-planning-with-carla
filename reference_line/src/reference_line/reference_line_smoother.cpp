#include "reference_line/reference_line_smoother.hpp"
#include "reference_line/reference_line_smooth_ipopt_interface.hpp"

namespace planning {

bool ReferenceLineSmoother::SmoothReferenceLine(
    const planning_srvs::RouteResponse &route_response,
    std::vector<ReferencePoint> *const smoothed_ref_points) {

  if (smoothed_ref_points == nullptr) {
    ROS_FATAL("[ReferenceLineSmoother::GetSmoothReferenceLine],"
              " the smoothed_ref_line is nullptr");
    return false;
  }

  route_response_ = route_response;
  const size_t point_num = route_response_.route.size();
  if (point_num < 3) {
    ROS_FATAL("[ReferenceLineSmoother::GetSmoothReferenceLine], "
              "the ref points num is less 3");
    return false;
  }

  std::vector<ReferencePoint> ref_points;
  ref_points.reserve(point_num);
  for (const auto &way_point : route_response_.route) {
    ReferencePoint ref_point;
    ref_point.set_xy(way_point.pose.position.x, way_point.pose.position.y);
    ref_points.push_back(ref_point);
  }

  ReferenceLineSmoothIpoptInterface smoother_interface(ref_points);
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

  smoothed_ref_points->clear();
  smoothed_ref_points->reserve(point_num);
  bool result = this->TraceSmoothReferenceLine(solution, smoothed_ref_points);
  return result;
}

bool ReferenceLineSmoother::SmoothReferenceLine(const std::vector<planning_msgs::WayPoint> &waypoints,
                                                std::vector<ReferencePoint> *const smoothed_ref_points) {
  if (smoothed_ref_points == nullptr) {
    ROS_FATAL("[ReferenceLineSmoother::GetSmoothReferenceLine],"
              " the smoothed_ref_line is nullptr");
    return false;
  }

  route_response_.route.reserve(waypoints.size());
  for (const auto &waypoint : waypoints) {
    route_response_.route.push_back(waypoint);
  }

  const size_t point_num = route_response_.route.size();
  if (point_num < 3) {
    ROS_FATAL("[ReferenceLineSmoother::GetSmoothReferenceLine], "
              "the ref points num is less 3");
    return false;
  }

  std::vector<ReferencePoint> ref_points;
  ref_points.reserve(point_num);
  for (const auto &way_point : route_response_.route) {
    ReferencePoint ref_point;
    ref_point.set_xy(way_point.pose.position.x, way_point.pose.position.y);
    ref_points.push_back(ref_point);
  }
  ReferenceLineSmoothIpoptInterface smoother_interface(ref_points);
  smoother_interface.set_ref_deviation_weight(deviation_weight_);
  smoother_interface.set_heading_weight(heading_weight_);
  smoother_interface.set_length_weight(distance_weight_);
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

  smoothed_ref_points->clear();
  smoothed_ref_points->reserve(waypoints.size());
  bool result = this->TraceSmoothReferenceLine(solution, smoothed_ref_points);
  return result;
}

bool ReferenceLineSmoother::SetUpConstraint() {
  const auto way_points = route_response_.route;
  size_t number_of_points = way_points.size();
  size_t number_of_curvature_constraints = number_of_points - 2;
  x_l_.resize(number_of_points * 2);
  x_u_.resize(number_of_points * 2);
  g_l_.resize(number_of_curvature_constraints);
  g_u_.resize(number_of_curvature_constraints);
  std::cout << "number_of_curvature_constraints: " << number_of_curvature_constraints << std::endl;
  for (size_t i = 0; i < number_of_points; ++i) {
    size_t index = i * 2;
    double boundary_radius = 1.4;
    x_l_[index] = way_points[i].pose.position.x - boundary_radius;
    x_u_[index] = way_points[i].pose.position.x + boundary_radius;
    x_l_[index + 1] = way_points[i].pose.position.y - boundary_radius;
    x_u_[index + 1] = way_points[i].pose.position.y + boundary_radius;
  }
  // for the last and begin point
  x_l_[0] = way_points[0].pose.position.x - 0.1;
  x_u_[0] = way_points[0].pose.position.x + 0.1;
  x_l_[1] = way_points[0].pose.position.y - 0.1;
  x_u_[1] = way_points[0].pose.position.y + 0.1;
  x_l_[2 * (number_of_points - 1)] = way_points.back().pose.position.x - 0.3;
  x_u_[2 * (number_of_points - 1)] = way_points.back().pose.position.x + 0.3;
  x_l_[2 * (number_of_points - 1) + 1] = way_points.back().pose.position.y - 0.3;
  x_u_[2 * (number_of_points - 1) + 1] = way_points.back().pose.position.y + 0.3;
  for (size_t i = 0; i < number_of_curvature_constraints; ++i) {
    g_l_[i] = -max_curvature_;
    g_u_[i] = max_curvature_;
  }

  return true;
}

void ReferenceLineSmoother::SetUpOptions() {
  options_.clear();
  options_ += "Integer print_level  0\n";
  options_ += "Sparse  true        forward\n";
  options_ += "Sparse  true        reverse\n";
//  options_ += "Numeric max_cpu_time  0.05\n";
  options_ += "Integer max_iter    12\n";
}

void ReferenceLineSmoother::SetUpInitValue() {
  const auto way_points = route_response_.route;
  size_t number_of_points = way_points.size();

  xi_.resize(number_of_points * 2);
  for (size_t i = 0; i < number_of_points; ++i) {
    size_t index = i * 2;
    xi_[index] = way_points[i].pose.position.x;
    xi_[index + 1] = way_points[i].pose.position.y;
  }
}

bool ReferenceLineSmoother::TraceSmoothReferenceLine(
    const CppAD::ipopt::solve_result<DVector> &result,
    std::vector<ReferencePoint> *const ref_points) const {

  if (result.status != CppAD::ipopt::solve_result<DVector>::success) {
    return false;
  }

  size_t number_of_points = route_response_.route.size();

  if (result.x.size() != number_of_points * 2) {
    return false;
  }

  ReferencePoint reference_point;
  ref_points->reserve(number_of_points);

  for (size_t i = 0; i < number_of_points; ++i) {
    size_t index = i * 2;
    reference_point.set_xy(result.x[index], result.x[index + 1]);
    ref_points->push_back(reference_point);
  }

  return true;
}
ReferenceLineSmoother::ReferenceLineSmoother(const double deviation_weight,
                                             const double heading_weight,
                                             const double distance_weight,
                                             const double max_curvature)
    : deviation_weight_(deviation_weight),
      heading_weight_(heading_weight),
      distance_weight_(distance_weight),
      max_curvature_(max_curvature) {}
void ReferenceLineSmoother::SetSmoothParams(const double deviation_weight,
                                            const double heading_weight,
                                            const double distance_weight,
                                            const double max_curvature) {
  deviation_weight_ = deviation_weight;
  heading_weight_ = heading_weight;
  distance_weight_ = distance_weight;
  max_curvature_ = max_curvature;
}

}