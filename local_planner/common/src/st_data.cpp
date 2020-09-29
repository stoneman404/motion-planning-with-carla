#include "st_data.hpp"
#include <ros/ros.h>
namespace planning {
// st point
STPoint::STPoint(double s, double t) : s_(s), t_(t) {}
STPoint::STPoint(const Eigen::Vector2d &vec_point) : s_(vec_point.x()), t_(vec_point.y()) {}
double STPoint::s() const { return s_; }
double STPoint::t() const { return t_; }
void STPoint::set_s(double s) { s_ = s; }
void STPoint::set_t(double t) { t_ = t; }

// st boundary
STBoundary::STBoundary(const std::vector<std::pair<STPoint, STPoint>> &point_pairs) {
  for (const auto &item : point_pairs) {
    const double t = item.first.t();
    lower_points_.emplace_back(item.first.s(), t);
    upper_points_.emplace_back(item.second.s(), t);
  }
  lower_left_point_ = lower_points_.front();
  lower_right_point_ = lower_points_.back();
  upper_left_point_ = upper_points_.front();
  upper_right_point_ = upper_points_.back();
  for (const auto &point : lower_points_) {
    points_.emplace_back(point);
  }
  for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); ++rit) {
    points_.emplace_back(*rit);
  }
  num_points_ = points_.size();
  ROS_ASSERT(num_points_ >= 3);
  area_ = 0.0;
  for (size_t i = 1; i < num_points_; ++i) {
    area_ += MathUtil::CrossProd({points_[0].t(), points_[0].s()},
                                 {points_[i - 1].t(), points_[i - 1].s()},
                                 {points_[i].t(), points_[i].s()});
  }
  if (area_ < 0) {
    area_ = -area_;
    std::reverse(points_.begin(), points_.end());
  }
  area_ /= 2.0;
  ROS_ASSERT(area_ > 1e-6);

  is_convex_ = true;
  for (int i = 0; i < num_points_; ++i) {
    if (MathUtil::CrossProd({points_[Prev(i)].t(), points_[Prev(i)].s()},
                            {points_[i].t(), points_[i].s()},
                            {points_[Next(i)].t(), points_[Next(i)].s()}) <=
        -1e-6) {
      is_convex_ = false;
      break;
    }
  }

  for (const auto &point : lower_points_) {
    min_s_ = std::fmin(min_s_, point.s());
  }
  for (const auto &point : upper_points_) {
    max_s_ = std::fmax(max_s_, point.s());
  }
  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();
}
size_t STBoundary::Prev(size_t i) const {
  return i >= num_points_ - 1 ? 0 : i + 1;
}
size_t STBoundary::Next(size_t i) const {
  return i <= 0 ? num_points_ - 1 : i - 1;
}
bool STBoundary::GetBoundarySRange(double curr_time, double *s_upper, double *s_lower) const {
  if (s_lower == nullptr || s_upper == nullptr) {
    return false;
  }
  if (curr_time < min_t_ || curr_time > max_t_) {
    return false;
  }
  size_t left = 0;
  size_t right = 0;
  GetIndexRange(lower_points_, curr_time, &left, &right);
  const double r =
      right == left ? 0.0 : (curr_time - upper_points_[left].t())
          / (upper_points_[right].t() - upper_points_[left].t());
  *s_upper = upper_points_[left].s() + r * (upper_points_[right].s() - upper_points_[left].s());
  *s_lower = lower_points_[left].s() + r * (lower_points_[right].s() - lower_points_[left].s());
  return true;
}
int STBoundary::id() const { return id_; }
double STBoundary::min_s() const { return min_s_; }
double STBoundary::min_t() const { return min_t_; }
double STBoundary::max_s() const { return max_s_; }
double STBoundary::max_t() const { return max_t_; }
const std::vector<STPoint> &STBoundary::upper_points() const { return upper_points_; }
const std::vector<STPoint> &STBoundary::lower_points() const { return lower_points_; }
const STPoint &STBoundary::upper_left_point() const { return upper_left_point_; }
const STPoint &STBoundary::upper_right_point() const { return upper_right_point_; }
const STPoint &STBoundary::lower_left_point() const { return lower_left_point_; }
const STPoint &STBoundary::lower_right_point() const { return lower_right_point_; }

/**
 *
 * @param curr_time
 * @param ds_upper
 * @param ds_lower
 * @return
 */
bool STBoundary::GetBoundarySlopes(double curr_time, double *ds_upper, double *ds_lower) const {
  if (ds_lower == nullptr || ds_upper == nullptr) {
    return false;
  }
  if (curr_time < min_t_ || curr_time > max_t_) {
    return false;
  }
  const double time_increment = 0.05;
  double t_prev = curr_time - time_increment;
  double prev_s_upper = 0.0;
  double prev_s_lower = 0.0;
  bool has_prev = GetBoundarySRange(t_prev, &prev_s_upper, &prev_s_lower);
  double t_next = curr_time + time_increment;
  double next_s_upper = 0.0;
  double next_s_lower = 0.0;
  bool has_next = GetBoundarySRange(t_next, &next_s_upper, &next_s_lower);
  double curr_s_upper = 0.0;
  double curr_s_lower = 0.0;
  GetBoundarySRange(curr_time, &curr_s_upper, &curr_s_lower);
  if (!has_next && !has_prev) {
    return false;
  }
  if (has_prev && has_next) {
    *ds_upper = ((next_s_upper - curr_s_upper) / time_increment +
        (curr_s_upper - prev_s_upper) / time_increment) *
        0.5;
    *ds_lower = ((next_s_lower - curr_s_lower) / time_increment +
        (curr_s_lower - prev_s_lower) / time_increment) *
        0.5;
    return true;
  }
  if (has_prev) {
    *ds_upper = (curr_s_upper - prev_s_upper) / time_increment;
    *ds_lower = (curr_s_lower - prev_s_lower) / time_increment;
  } else {
    *ds_upper = (next_s_upper - curr_s_upper) / time_increment;
    *ds_lower = (next_s_lower - curr_s_lower) / time_increment;
  }
  return true;
}

bool STBoundary::GetIndexRange(const std::vector<STPoint> &points, double t, size_t *left_index, size_t *right_index) {
  if (left_index == nullptr || right_index == nullptr) {
    return false;
  }
  if (t < points.front().t() || t > points.back().t()) {
    ROS_DEBUG("[STBoundary::GetIndexRange], t is out of range: t=%f", t);
    return false;
  }
  auto comp = [](const STPoint &p, double t) { return p.t() < t; };
  auto first_it = std::lower_bound(points.begin(), points.end(), t, comp);
  size_t index = std::distance(points.begin(), first_it);
  if (index == 0) {
    *left_index = *right_index = 0;
  } else if (index == points.size() - 1) {
    *left_index = *right_index = points.size() - 1;
  } else {
    *left_index = index - 1;
    *right_index = index;
  }
  return true;
}
bool STBoundary::IsPointInBoundary(const STPoint &st_point) const {
  if (st_point.t() <= min_t_ || st_point.t() >= max_t_) {
    return false;
  }
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, st_point.t(), &left, &right)) {
    return false;
  }
  const double check_upper = MathUtil::CrossProd({st_point.t(), st_point.s()},
                                                 {upper_points_[left].t(), upper_points_[left].s()},
                                                 {upper_points_[right].t(), upper_points_[right].s()});
  const double check_lower = MathUtil::CrossProd({st_point.t(), st_point.s()},
                                                 {lower_points_[left].t(), lower_points_[left].s()},
                                                 {lower_points_[right].t(), lower_points_[right].s()});
  return check_lower * check_upper < 0.0;
}

}


