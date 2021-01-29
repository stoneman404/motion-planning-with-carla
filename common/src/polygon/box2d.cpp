#include <ros/ros.h>
#include "polygon/box2d.hpp"
#include "math/math_utils.hpp"
#include <Eigen/Core>
#include <utility>
namespace common {

Box2d::Box2d(Eigen::Vector2d center,
             double heading,
             double length,
             double width)
    : center_(std::move(center)),
      heading_(heading),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0),
      cos_heading_(std::cos(heading)),
      sin_heading_(std::sin(heading)) {
  assert(length_ > -std::numeric_limits<double>::epsilon());
  assert(width_ > -std::numeric_limits<double>::epsilon());
  InitCorners();

}
void Box2d::InitCorners() {
  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  corners_.clear();
  corners_.emplace_back(center_.x() + dx1 + dx2, center_.y() + dy1 + dy2);
  corners_.emplace_back(center_.x() + dx1 - dx2, center_.y() + dy1 - dy2);
  corners_.emplace_back(center_.x() - dx1 - dx2, center_.y() - dy1 - dy2);
  corners_.emplace_back(center_.x() - dx1 + dx2, center_.y() - dy1 + dy2);

  for (auto &corner : corners_) {
    max_x_ = std::fmax(corner.x(), max_x_);
    min_x_ = std::fmin(corner.x(), min_x_);
    max_y_ = std::fmax(corner.y(), max_y_);
    min_y_ = std::fmin(corner.y(), min_y_);
  }
}

std::vector<Eigen::Vector2d> Box2d::GetAllCorners() const {
  return corners_;
}

bool Box2d::IsPointIn(const Eigen::Vector2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::fabs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::fabs(-x0 * sin_heading_ + y0 * cos_heading_);
  return dx <= half_length_ + 1e-9 && dy <= half_width_ + 1e-9;
}

bool Box2d::IsPointOnBoundary(const Eigen::Vector2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
  return (std::abs(dx - half_length_) <= 1e-9 &&
      dy <= half_width_ + 1e-9) ||
      (std::abs(dy - half_width_) <= 1e-9 &&
          dx <= half_length_ + 1e-9);
}

double Box2d::DistanceToPoint(const Eigen::Vector2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
  const double dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

// ref: https://zhuanlan.zhihu.com/p/146778379
// ref: apollo
bool Box2d::HasOverlapWithBox2d(const common::Box2d &box) const {
  if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() ||
      box.min_y() > max_y()) {
    return false;
  }

  const double shift_x = box.center_x() - center_.x();
  const double shift_y = box.center_y() - center_.y();

  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  const double dx3 = box.cos_heading() * box.half_length();
  const double dy3 = box.sin_heading() * box.half_length();
  const double dx4 = box.sin_heading() * box.half_width();
  const double dy4 = -box.cos_heading() * box.half_width();

  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
      std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
          std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
          half_length_ &&
      std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
          std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
              std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
              half_width_ &&
      std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
          std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
              std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
              box.half_length() &&
      std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
          std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
              std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
              box.half_width();
}

void Box2d::RotateFromCenter(const double rotate_angle) {
  heading_ = MathUtils::NormalizeAngle(heading_ + rotate_angle);
  cos_heading_ = std::cos(heading_);
  sin_heading_ = std::sin(heading_);
  InitCorners();
}

void Box2d::Shift(const Eigen::Vector2d &shift_vec) {
  center_ += shift_vec;
  InitCorners();

}

void Box2d::LongitudinalExtend(const double extension_length) {
  length_ += extension_length;
  half_length_ += extension_length / 2.0;
  InitCorners();
}

void Box2d::LateralExtend(const double extension_length) {
  width_ += extension_length;
  half_width_ += extension_length / 2.0;
  InitCorners();
}
const Eigen::Vector2d &Box2d::Center() const { return center_; }
double Box2d::center_x() const { return center_.x(); }
double Box2d::center_y() const { return center_.y(); }
double Box2d::length() const { return length_; }
double Box2d::width() const { return width_; }
double Box2d::half_length() const { return half_length_; }
double Box2d::half_width() const { return half_width_; }
double Box2d::heading() const { return heading_; }
double Box2d::cos_heading() const { return cos_heading_; }
double Box2d::sin_heading() const { return sin_heading_; }
double Box2d::area() const { return length_ * width_; }
double Box2d::diagonal() const { return std::hypot(length_, width_); }
double Box2d::max_x() const { return max_x_; }
double Box2d::min_x() const { return min_x_; }
double Box2d::max_y() const { return max_y_; }
double Box2d::min_y() const { return min_y_; }

}