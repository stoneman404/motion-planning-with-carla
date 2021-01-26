#include "reference_line/reference_line_smooth_ipopt_interface.hpp"

#include <ros/ros.h>

namespace planning {

void ReferenceLineSmoothIpoptInterface::operator()(
    ReferenceLineSmoothIpoptInterface::ADvector &fg,
    const ReferenceLineSmoothIpoptInterface::ADvector &x) {
//  size_t point_num = ref_points_.size();
  assert(x.size() == num_of_variables_);
//  assert(fg.size() == 1);
  assert(fg.size() == num_of_constraint_ + 1);
  fg[0] = 0.0;
  // deviation from origin reference line
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i * 2;
    fg[0] += ref_deviation_weight_ * ((x[index] - ref_points_[i].first) * (x[index] - ref_points_[i].first)
        + (x[index + 1] - ref_points_[i].second) * (x[index + 1] - ref_points_[i].second));
  }
  // the theta error cost;
  for (size_t i = 0; i < num_of_points_ - 2; ++i) {
    size_t findex = i * 2;
    size_t mindex = findex + 2;
    size_t lindex = mindex + 2;
    fg[0] += heading_weight_ * (CppAD::pow((x[findex] + x[lindex] - 2.0 * x[mindex]), 2) +
        CppAD::pow((x[findex + 1] + x[lindex + 1] - 2.0 * x[mindex + 1]), 2));
  }
  // the total length cost
  for (size_t i = 0; i < num_of_points_ - 1; ++i) {
    size_t findex = i * 2;
    size_t nindex = findex + 2;
    fg[0] += length_weight_ * (CppAD::pow(x[findex] - x[nindex], 2) +
        CppAD::pow(x[findex + 1] - x[nindex + 1], 2));
  }

  for (size_t i = slack_variable_start_index_; i < slack_variable_end_index_; ++i) {
    fg[0] += slack_weight_ * x[i];
  }

  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i * 2;
    fg[index + 1] = x[index];
    fg[index + 2] = x[index + 1];
  }
  // the constraint function
  for (size_t i = 0; i + 2 < num_of_points_; ++i) {
    size_t findex = i * 2;
    size_t mindex = findex + 2;
    size_t lindex = mindex + 2;

    fg[curvature_constraint_start_index_ + 1 + i] = (((x[findex] + x[lindex]) - 2.0 * x[mindex]) *
        ((x[findex] + x[lindex]) - 2.0 * x[mindex]) +
        ((x[findex + 1] + x[lindex + 1]) - 2.0 * x[mindex + 1]) *
            ((x[findex + 1] + x[lindex + 1]) - 2.0 * x[mindex + 1])) - x[slack_variable_start_index_ + i];
  }

  size_t slack_var_index = 0;
  for (size_t i = slack_constraint_start_index_; i < slack_constraint_end_index_; ++i) {
    fg[i + 1] = x[slack_variable_start_index_ + slack_var_index];
    ++slack_var_index;
  }
}

ReferenceLineSmoothIpoptInterface::ReferenceLineSmoothIpoptInterface(const std::vector<std::pair<double,
                                                                                                 double>> &ref_points) {
  assert(ref_points.size() >= 3);
  ref_points_ = ref_points;
  num_of_points_ = ref_points_.size();
  num_of_slack_variable_ = num_of_points_ - 2;
  num_of_variables_ = num_of_points_ * 2 + num_of_slack_variable_;
  num_curvature_constraint_ = num_of_points_ - 2;
  slack_variable_start_index_ = num_of_points_ * 2;
  slack_variable_end_index_ = slack_variable_start_index_ + num_of_slack_variable_;

  num_of_slack_constr_ = num_of_points_ - 2;
  num_of_constraint_ = num_of_points_ * 2 + num_of_slack_constr_ + num_curvature_constraint_;
  curvature_constraint_start_index_ = num_of_points_ * 2;
  curvature_constraint_end_index_ = curvature_constraint_start_index_ + num_curvature_constraint_;
  slack_constraint_start_index_ = curvature_constraint_end_index_;
  slack_constraint_end_index_ = slack_constraint_start_index_ + num_of_slack_constr_;

}
}
