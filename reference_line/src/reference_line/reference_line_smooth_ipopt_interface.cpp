#include "reference_line/reference_line_smooth_ipopt_interface.hpp"

#include <ros/ros.h>

namespace planning {

void ReferenceLineSmoothIpoptInterface::operator()(
    ReferenceLineSmoothIpoptInterface::ADvector &fg,
    const ReferenceLineSmoothIpoptInterface::ADvector &x) {
  assert(x.size() == 2 * number_of_points_);
//  assert(fg.size() == 1);
  assert(fg.size() == number_of_curvature_constraints_ + 1);
  fg[0] = 0.0;
  // deviation from origin reference line
  for (size_t i = 0; i < number_of_points_; ++i) {
    size_t index = i * 2;
    fg[0] += ref_deviation_weight_ * ((x[index] - ref_points_[i].x()) * (x[index] - ref_points_[i].x())
        + (x[index + 1] - ref_points_[i].y()) * (x[index + 1] - ref_points_[i].y()));
//    std::cout << "deviation_distance : "
//              << ref_deviation_weight_ * ((x[index] - ref_points_[i].x()) * (x[index] - ref_points_[i].x())
//                  + (x[index + 1] - ref_points_[i].y()) * (x[index + 1] - ref_points_[i].y()))
//              << std::endl;
  }
  // the theta error cost;
  for (size_t i = 0; i < number_of_points_ - 2; ++i) {
    size_t findex = i * 2;
    size_t mindex = findex + 2;
    size_t lindex = mindex + 2;
    fg[0] += heading_weight_ * (CppAD::pow((x[findex] + x[lindex] - 2.0 * x[mindex]), 2) +
        CppAD::pow((x[findex + 1] + x[lindex + 1] - 2.0 * x[mindex + 1]), 2));
//    std::cout << "theta cost : "
//              << heading_weight_ * (CppAD::pow((x[findex] + x[lindex] - 2.0 * x[mindex]), 2) +
//                  CppAD::pow((x[findex + 1] + x[lindex + 1] - 2.0 * x[mindex + 1]), 2))
//              << std::endl;
  }
  // the total length cost
  for (size_t i = 0; i < number_of_points_ - 1; ++i) {
    size_t findex = i * 2;
    size_t nindex = findex + 2;
    fg[0] += length_weight_ * (CppAD::pow(x[findex] - x[nindex], 2) +
        CppAD::pow(x[findex + 1] - x[nindex + 1], 2));
  }
  // the curvature cost
//  for (size_t i = 0; i < number_of_points_ - 2; ++i) {
//    size_t findex = i * 2;
//    size_t mindex = findex + 2;
//    size_t lindex = mindex + 2;
//    CppAD::AD<double> dx_i = x[mindex] - x[findex];
//    CppAD::AD<double> dy_i = x[mindex + 1] - x[findex + 1];
//    CppAD::AD<double> dx_ip1 = x[lindex] - x[mindex];
//    CppAD::AD<double> dy_ip1 = x[lindex + 1] - x[mindex + 1];
//    CppAD::AD<double> dxy_i_dot_dxy_ip1 = dx_i * dx_ip1 + dy_i * dy_ip1 ;
//    CppAD::AD<double> dxy_i_norm = CppAD::sqrt(dx_i * dx_i + dy_i * dy_i) + 0.01;
//    CppAD::AD<double> dxy_ip1_norm = CppAD::sqrt(dx_ip1 * dx_ip1 + dy_ip1 * dy_ip1) + 0.01;
//    CppAD::AD<double> d_theta_i = CppAD::acos(dxy_i_dot_dxy_ip1 / (dxy_i_norm * dxy_ip1_norm));
//    fg[0] += curvature_weight_ * CppAD::pow(d_theta_i / dxy_i_norm, 2);
////    std::cout << "curvature_weight_ * CppAD::pow(d_theta_i / dxy_i_norm, 2): "
////              << curvature_weight_ * CppAD::pow(d_theta_i / dxy_i_norm, 2) << std::endl;
//  }
//  std::cout << "=======fg[0]: " << fg[0] << "========" << std::endl;
  // the constraint function
  for (size_t i = 0; i < number_of_curvature_constraints_; ++i) {
    size_t findex = i * 2;
    size_t mindex = findex + 2;
    size_t lindex = mindex + 2;
//    fg[i + 1] =  (CppAD::pow((x[findex] + x[lindex] - 2.0 * x[mindex]), 2) +
//        CppAD::pow((x[findex + 1] + x[lindex + 1] - 2.0 * x[mindex + 1]), 2));
    CppAD::AD<double> dx_i = x[mindex] - x[findex];
    CppAD::AD<double> dy_i = x[mindex + 1] - x[findex + 1];
    CppAD::AD<double> dx_ip1 = x[lindex] - x[mindex];
    CppAD::AD<double> dy_ip1 = x[lindex + 1] - x[mindex + 1];
    CppAD::AD<double> dxy_i_dot_dxy_ip1 = dx_i * dx_ip1 + dy_i * dy_ip1;
    CppAD::AD<double> dxy_i_norm = CppAD::sqrt(dx_i * dx_i + dy_i * dy_i);
    CppAD::AD<double> dxy_ip1_norm = CppAD::sqrt(dx_ip1 * dx_ip1 + dy_ip1 * dy_ip1) + 0.01;
    CppAD::AD<double> d_theta_i = CppAD::acos(dxy_i_dot_dxy_ip1 / (dxy_i_norm * dxy_ip1_norm)) + 0.01;
    fg[i + 1] = d_theta_i / dxy_i_norm;
    fg[i + 1] = (((x[findex] + x[lindex]) - 2.0 * x[mindex]) *
        ((x[findex] + x[lindex]) - 2.0 * x[mindex]) +
        ((x[findex + 1] + x[lindex + 1]) - 2.0 * x[mindex + 1]) *
            ((x[findex + 1] + x[lindex + 1]) - 2.0 * x[mindex + 1]));
  }
}

ReferenceLineSmoothIpoptInterface::ReferenceLineSmoothIpoptInterface(const std::vector<ReferencePoint> &ref_points) {
  assert(ref_points.size() >= 3);
  ref_points_ = ref_points;
  number_of_points_ = ref_points.size();
  number_of_variables_ = 2 * number_of_points_;
  number_of_curvature_constraints_ = number_of_points_ - 2;
  number_of_constraints_ = number_of_variables_ + number_of_curvature_constraints_;
//  std::cout << "===============params : ===============" << std::endl;
//  std::cout << "curvature_weight_ : " << curvature_weight_ << std::endl;
//  std::cout << "heading_weight_ : " << heading_weight_ << std::endl;
//  std::cout << "length_weight_ : " << length_weight_ << std::endl;
//  std::cout << "ref_deviation_weight_ : " << ref_deviation_weight_ << std::endl;

}
}
