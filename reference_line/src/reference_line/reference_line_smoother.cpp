#include "reference_line/reference_line_smoother.hpp"
#include "reference_line/reference_line_smooth_ipopt_interface.hpp"


namespace planning {

bool ReferenceLineSmoother::SmoothReferenceLine(const std::vector<ReferencePoint> &raw_points,
                                                std::vector<ReferencePoint> *const smoothed_ref_points) {
  if (smoothed_ref_points == nullptr) {
    ROS_FATAL("[ReferenceLineSmoother::GetSmoothReferenceLine],"
              " the smoothed_ref_line is nullptr");
    return false;
  }

//  way_points.route.reserve(waypoints.size());
  ref_points_ = raw_points;

  const size_t point_num = ref_points_.size();
  if (point_num < 3) {
    ROS_FATAL("[ReferenceLineSmoother::GetSmoothReferenceLine], "
              "the ref points num is less 3");
    return false;
  }

  num_of_points_ = point_num;
  num_of_slack_variable_ = num_of_points_ - 2;
  num_of_variables_ = num_of_points_ * 2 + num_of_slack_variable_;
  num_curvature_constraint_ = num_of_points_ - 2;
  slack_variable_start_index_ = num_of_points_ * 2;
  slack_variable_end_index_ = slack_variable_start_index_ + num_of_slack_variable_;

  num_of_constraint_ = num_curvature_constraint_;
//  num_of_slack_constr_ = num_of_points_ - 2;
//  num_of_constraint_ = num_of_points_ * 2 + num_of_slack_constr_ + num_curvature_constraint_;
  curvature_constraint_start_index_ = 0;
  curvature_constraint_end_index_ = curvature_constraint_start_index_ + num_curvature_constraint_;
//  slack_constraint_start_index_ = curvature_constraint_end_index_;
//  slack_constraint_end_index_ = slack_constraint_start_index_ + num_of_slack_constr_;
  std::vector<std::pair<double, double>> xy;
  for (const auto &ref_point : ref_points_) {
    xy.emplace_back(ref_point.x(), ref_point.y());
  }
  ReferenceLineSmoothIpoptInterface smoother_interface(xy);
  smoother_interface.set_ref_deviation_weight(deviation_weight_);
  smoother_interface.set_heading_weight(heading_weight_);
  smoother_interface.set_length_weight(distance_weight_);
  smoother_interface.set_slack_weight(slack_weight_);
  SetUpOptions();
  SetUpInitValue();
  if (!this->SetUpConstraint()) {
    std::cout << "set up constraint error" << std::endl;
    return false;
  }
  CppAD::ipopt::solve_result<DVector> solution;
  CppAD::ipopt::solve<DVector, ReferenceLineSmoothIpoptInterface>(options_, xi_, x_l_, x_u_, g_l_, g_u_,
                                                                  smoother_interface, solution);
  if (solution.status != CppAD::ipopt::solve_result<DVector>::success) {
    printf("[GetSmoothReferenceLine] failed reason: %i",
           solution.status);
    return false;
  }

  smoothed_ref_points->clear();
  smoothed_ref_points->reserve(raw_points.size());
  bool result = this->TraceSmoothReferenceLine(solution, smoothed_ref_points);
  return result;
}

bool ReferenceLineSmoother::SetUpConstraint() {

  x_l_.resize(num_of_variables_);
  x_u_.resize(num_of_variables_);
  g_l_.resize(num_of_constraint_);
  g_u_.resize(num_of_constraint_);
  double boundary_radius = 1.0;

//  std::cout << "number_of_curvature_constraints: " << number_of_curvature_constraints << std::endl;
  std::vector<double> boundary_bound(num_of_points_, 0.0);
  for (size_t i = 0; i < num_of_points_; ++i) {
    if (i == 0 || i == num_of_points_ - 1) {
      boundary_bound[i] = 0.4;
    } else {
      boundary_bound[i] = boundary_radius;
    }
  }
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i * 2;

    x_l_[index] = ref_points_[i].x() - boundary_bound[i];
    x_u_[index] = ref_points_[i].x() + boundary_bound[i];
    x_l_[index + 1] = ref_points_[i].y() - boundary_bound[i];
    x_u_[index + 1] = ref_points_[i].y() + boundary_bound[i];
  }

  // slack variable
  for (size_t i = slack_variable_start_index_; i < slack_variable_end_index_; ++i) {
    x_l_[i] = 0.0;
    x_u_[i] = 1e20;
  }

  // calculate curvature upper constraints
  double ref_line_total_length = 0.0;
  for (int i = 1; i < num_of_points_; ++i) {
    const auto last_ref_point = ref_points_[i - 1];
    const auto cur_ref_point = ref_points_[i];
    ref_line_total_length += hypot(cur_ref_point.x() - last_ref_point.x(), cur_ref_point.y() - last_ref_point.y());
  }

  double average_ds = ref_line_total_length / static_cast<double>(num_of_points_ - 1);
  double curvature_upper = average_ds * average_ds * max_curvature_;

  for (size_t i = curvature_constraint_start_index_; i < curvature_constraint_end_index_; ++i) {
    g_l_[i] = -1e20;
    g_u_[i] = curvature_upper * curvature_upper;
  }
  return true;
}

void ReferenceLineSmoother::SetUpOptions() {
  options_.clear();
  options_ += "Integer print_level  0\n";
  options_ += "Sparse  true        reverse\n";
  options_ += "Numeric tol          1e-5\n";
  options_ += "Integer max_iter    15\n";
}

void ReferenceLineSmoother::SetUpInitValue() {

  xi_.resize(num_of_variables_);
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i * 2;
    xi_[index] = ref_points_[i].x();
    xi_[index + 1] = ref_points_[i].y();
  }
  // slack variables
  for (size_t i = slack_variable_start_index_; i < slack_variable_end_index_; ++i) {
    xi_[i] = .0;
  }
}

bool ReferenceLineSmoother::TraceSmoothReferenceLine(
    const CppAD::ipopt::solve_result<DVector> &result,
    std::vector<ReferencePoint> *const ref_points) const {

  if (result.status != CppAD::ipopt::solve_result<DVector>::success) {
    std::cout << " result != success" << std::endl;
    return false;
  }

  if (result.x.size() != num_of_variables_) {
    return false;
  }

  ReferencePoint reference_point;
  ref_points->reserve(num_of_points_);

  for (size_t i = 0; i < num_of_points_; ++i) {
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
      max_curvature_(max_curvature) {

}

void ReferenceLineSmoother::SetSmoothParams(double deviation_weight,
                                            double distance_weight,
                                            double heading_weight,
                                            double slack_weight,
                                            double max_curvature) {
  deviation_weight_ = deviation_weight;
  heading_weight_ = heading_weight;
  distance_weight_ = distance_weight;
  max_curvature_ = max_curvature;
  slack_weight_ = slack_weight;
}

}