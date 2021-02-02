#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include "reference_line/reference_line.hpp"
#include "math/math_utils.hpp"
#include "math/coordinate_transformer.hpp"

namespace planning {
using namespace common;
ReferenceLine::ReferenceLine(const std::vector<planning_msgs::WayPoint> &waypoints)
    : way_points_(waypoints) {
  ROS_ASSERT(waypoints.size() >= 3);
  reference_smoother_ = std::make_shared<ReferenceLineSmoother>();
  size_t waypoints_size = waypoints.size();
  reference_points_.reserve(waypoints_size);
  left_boundary_.reserve(waypoints_size);
  right_boundary_.reserve(waypoints_size);
  for (size_t i = 0; i < waypoints_size; ++i) {
    const auto way_point = waypoints[i];
    const double left_width = way_point.lane_width / 2.0;
    const double right_width = way_point.lane_width / 2.0;
    Eigen::Vector2d heading;
    if (i + 1 >= waypoints.size()) {
      heading = Eigen::Vector2d(waypoints[i].pose.position.x -
                                    waypoints[i - 1].pose.position.x,
                                waypoints[i].pose.position.y -
                                    waypoints[i - 1].pose.position.y);
    } else {
      heading = Eigen::Vector2d(waypoints[i + 1].pose.position.x -
                                    waypoints[i].pose.position.x,
                                waypoints[i + 1].pose.position.y -
                                    waypoints[i].pose.position.y);
    }
    auto ref_point = ReferencePoint(way_point.pose.position.x, way_point.pose.position.y);
    ref_point.set_xy(way_point.pose.position.x, way_point.pose.position.y);
    ref_point.set_theta(MathUtils::NormalizeAngle(std::atan2(heading(1), heading(0))));
    reference_points_.push_back(ref_point);
    left_boundary_.emplace_back(
        ref_point.x() - left_width * std::sin(ref_point.theta()),
        ref_point.y() + left_width * std::cos(ref_point.theta()));
    right_boundary_.emplace_back(
        ref_point.x() + right_width * std::sin(ref_point.theta()),
        ref_point.y() + right_width * std::cos(ref_point.theta()));
  }

  ROS_ASSERT(reference_points_.size() == waypoints.size());
  ROS_ASSERT(left_boundary_.size() == waypoints.size());
  ROS_ASSERT(right_boundary_.size() == waypoints.size());
  bool result = BuildReferenceLineWithSpline();
  ROS_ASSERT(result);
  length_ = ref_line_spline_->ArcLength();
  ROS_INFO("ReferenceLine's length : %lf", length_);
}

FrenetFramePoint ReferenceLine::GetFrenetFramePoint(
    const planning_msgs::PathPoint &path_point) const {
  if (reference_points_.empty()) {
    return FrenetFramePoint();
  }
  SLPoint sl;
  XYToSL(path_point.x, path_point.y, &sl);
  FrenetFramePoint frame_point;
  frame_point.s = sl.s;
  frame_point.l = sl.l;
  const double theta = path_point.theta;
  const double kappa = path_point.kappa;
  const double l = frame_point.l;
  ReferencePoint ref_point = GetReferencePoint(frame_point.s);
  const double theta_ref = ref_point.theta();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();
  const double dl = CoordinateTransformer::CalcLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const double ddl = CoordinateTransformer::CalcSecondOrderLateralDerivative(
      theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frame_point.dl = dl;
  frame_point.ddl = ddl;
  return frame_point;
}

ReferencePoint ReferenceLine::GetReferencePoint(double s) const {
  double ref_x, ref_y;
  double ref_dx, ref_dy;
  double ref_ddx, ref_ddy;
  double ref_dddx, ref_dddy;
  ref_line_spline_->Evaluate(s, &ref_x, &ref_y);
  ref_line_spline_->EvaluateFirstDerivative(s, &ref_dx, &ref_dy);
  ref_line_spline_->EvaluateSecondDerivative(s, &ref_ddx, &ref_ddy);
  ref_line_spline_->EvaluateThirdDerivative(s, &ref_dddx, &ref_dddy);
  double ref_heading = MathUtils::NormalizeAngle(std::atan2(ref_dy, ref_dx));
  double ref_kappa = MathUtils::CalcKappa(ref_dx, ref_dy, ref_ddx, ref_ddy);
  double ref_dkappa = MathUtils::CalcDKappa(ref_dx, ref_dy, ref_ddx, ref_ddy, ref_dddx, ref_dddy);
  return ReferencePoint(ref_x, ref_y, ref_heading, ref_kappa, ref_dkappa);
}

ReferencePoint ReferenceLine::GetReferencePoint(double x, double y) const {
  ROS_ASSERT(!reference_points_.empty());
  double nearest_x, nearest_y, nearest_s;
  ref_line_spline_->GetNearestPointOnSpline(x, y, &nearest_x, &nearest_y, &nearest_s);
  double ref_dx, ref_dy;
  double ref_ddx, ref_ddy;
  double ref_dddx, ref_dddy;
  ref_line_spline_->EvaluateFirstDerivative(nearest_s, &ref_dx, &ref_dy);
  ref_line_spline_->EvaluateSecondDerivative(nearest_s, &ref_ddx, &ref_ddy);
  ref_line_spline_->EvaluateThirdDerivative(nearest_s, &ref_dddx, &ref_dddy);
  double heading = MathUtils::NormalizeAngle(std::atan2(ref_dy, ref_dx));
  double kappa = MathUtils::CalcKappa(ref_dx, ref_dy, ref_ddx, ref_ddy);
  double dkappa = MathUtils::CalcDKappa(ref_dx, ref_dy, ref_ddx, ref_ddy, ref_dddx, ref_dddy);
  return ReferencePoint(nearest_x, nearest_y, heading, kappa, dkappa);
}

ReferencePoint ReferenceLine::GetReferencePoint(const std::pair<double, double> &xy) const {

  const double x = xy.first;
  const double y = xy.second;
  return GetReferencePoint(x, y);
}

bool ReferenceLine::GetLaneWidth(double s, double *const left_width, double *const right_width) const {

  auto matched_ref_point = GetReferencePoint(s);
  const double x = matched_ref_point.x();
  const double y = matched_ref_point.y();
  double left_nearest_x, left_nearest_y, right_nearest_x, right_nearest_y, nearest_s;
  if (!left_boundary_spline_->GetNearestPointOnSpline(
      x, y, &left_nearest_x, &left_nearest_y, &nearest_s)) {
    return false;
  }
  if (!right_boundary_spline_->GetNearestPointOnSpline(
      x, y, &right_nearest_x, &right_nearest_y, &nearest_s)) {
    return false;
  }

  *left_width = std::hypot(left_nearest_x - x, left_nearest_y - y);
  *right_width = std::hypot(right_nearest_x - x, right_nearest_y - y);

  return true;
}

double ReferenceLine::Length() const {
  return length_;
}

bool ReferenceLine::IsOnLane(const SLPoint &sl_point) const {
  if (sl_point.s < 0 || sl_point.s > Length()) {
    return false;
  }
  double left_width, right_width;
  if (!this->GetLaneWidth(sl_point.s, &left_width, &right_width)) {
    return false;
  }
  return sl_point.l <= left_width && sl_point.l >= -right_width;
}

bool ReferenceLine::IsOnLane(const SLBoundary &sl_boundary) const {

  // if the obstacle is out of range
  if (sl_boundary.end_s < 0 || sl_boundary.start_s > Length()) {
    return false;
  }
  double middle_s = 0.5 * (sl_boundary.start_s + sl_boundary.end_s);
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!this->GetLaneWidth(middle_s, &lane_left_width, &lane_right_width)) {
    return false;
  }
  // satisfy this condition means the obstacle has influence on current lane
  return sl_boundary.start_l <= lane_left_width && sl_boundary.end_l >= -lane_right_width;
}

bool ReferenceLine::GetSLBoundary(const Box2d &box, SLBoundary *sl_boundary) const {

  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<Eigen::Vector2d> corners = box.GetAllCorners();
  // The order must be counter-clockwise
  std::vector<SLPoint> sl_corners;
  for (const auto &point : corners) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      return false;
    }
    sl_corners.push_back(std::move(sl_point));
  }

  for (size_t i = 0; i < corners.size(); ++i) {
    auto index0 = i;
    auto index1 = (i + 1) % corners.size();
    const auto &p0 = corners[index0];
    const auto &p1 = corners[index1];

    const auto p_mid = (p0 + p1) * 0.5;
    SLPoint sl_point_mid;
    if (!XYToSL(p_mid, &sl_point_mid)) {
      return false;
    }

    Eigen::Vector2d v0(sl_corners[index1].s - sl_corners[index0].s,
                       sl_corners[index1].l - sl_corners[index0].l);

    Eigen::Vector2d v1(sl_point_mid.s - sl_corners[index0].s,
                       sl_point_mid.l - sl_corners[index0].l);

    sl_boundary->boundary_points.push_back(sl_corners[index0]);

    // sl_point is outside of polygon; add to the vertex list
    double cross_prod = v0.x() * v1.y() - v0.y() * v1.x();
    if (cross_prod < 0.0) {
      sl_boundary->boundary_points.push_back(sl_point_mid);
    }
  }

  for (const auto &sl_point : sl_boundary->boundary_points) {
    start_s = std::fmin(start_s, sl_point.s);
    end_s = std::fmax(end_s, sl_point.s);
    start_l = std::fmin(start_l, sl_point.l);
    end_l = std::fmax(end_l, sl_point.l);
  }

  sl_boundary->start_s = start_s;
  sl_boundary->end_s = end_s;
  sl_boundary->start_l = start_l;
  sl_boundary->end_l = end_l;
  return true;
}

ReferencePoint ReferenceLine::Interpolate(const ReferencePoint &p0,
                                          const ReferencePoint &p1,
                                          double s0,
                                          double s1,
                                          double s) {
  double x = MathUtils::lerp(p0.x(), s0, p1.x(), s1, s);
  double y = MathUtils::lerp(p0.y(), s0, p1.y(), s1, s);
  double heading = MathUtils::slerp(p0.theta(), s0, p1.theta(), s1, s);
  double kappa = MathUtils::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  double dkappa = MathUtils::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);
  auto ref_point = ReferencePoint(x, y, heading, kappa, dkappa);

  return ref_point;
}

bool ReferenceLine::XYToSL(const Eigen::Vector2d &xy, SLPoint *sl_point) const {
  double nearest_x, nearest_y, nearest_s;
//  std::cout << "ReferenceLine::XYToSL: " << xy.x() << " , " << xy.y() << std::endl;
//  std::cout << "XYToSL: referenceline's arc length: " << ref_line_spline_->ArcLength();
  if (!ref_line_spline_->GetNearestPointOnSpline(
      xy(0), xy(1), &nearest_x, &nearest_y, &nearest_s)) {
    ROS_FATAL("[ReferenceLine::XYToSL], Failed to Get NearestPointOnSpline {x: %lf, y:%lf}", xy(0), xy(1));
    return false;
  }
  double x_der, y_der;
  Eigen::Vector2d heading;
//  ref_line_spline_->EvaluateFirstDerivative(nearest_s, &x_der, &y_der);
  auto ref_point = this->GetReferencePoint(nearest_s);
  heading << std::cos(ref_point.theta()), std::sin(ref_point.theta());
  Eigen::Vector2d vec;
  vec << xy(0) - nearest_x, xy(1) - nearest_y;
  double prod = heading(0) * vec(1) - heading(1) * vec(0);
  double proj = heading(0) * vec(0) + heading(1) * vec(1);
  if (nearest_s > 1e-3 || nearest_s < ref_line_spline_->ArcLength() - 1e-3) {
    sl_point->l = prod < 0 ?
                  -1.0 * std::hypot(xy(0) - nearest_x, xy(1) - nearest_y)
                           : std::hypot(xy(0) - nearest_x,
                                        xy(1) - nearest_y);

    sl_point->s = nearest_s;
  } else if (nearest_s < 1e-3) {
    sl_point->s = std::min(proj, nearest_s);
    if (proj < 0.0) {
      sl_point->l = prod;
    } else {
      sl_point->l = prod < 0 ?
                    -1.0 * std::hypot(xy(0) - nearest_x, xy(1) - nearest_y)
                             : std::hypot(xy(0) - nearest_x,
                                          xy(1) - nearest_y);
    }
  } else {
    sl_point->s = nearest_s + std::max(0.0, proj);
    if (proj > 0) {
      sl_point->l = prod;
    } else {
      sl_point->l = prod < 0 ?
                    -1.0 * std::hypot(xy(0) - nearest_x, xy(1) - nearest_y)
                             : std::hypot(xy(0) - nearest_x,
                                          xy(1) - nearest_y);
    }
  }
  return true;

}

bool ReferenceLine::XYToSL(double x, double y, SLPoint *sl_point) const {

  Eigen::Vector2d xy;
  xy << x, y;
  return XYToSL(xy, sl_point);

}

bool ReferenceLine::SLToXY(const SLPoint &sl_point, Eigen::Vector2d *xy_point) const {
  if (reference_points_.size() < 2) {
    return false;
  }
  const auto matched_ref_point = GetReferencePoint(sl_point.s);
  const auto angle = matched_ref_point.theta();
  const double cos_theta = std::cos(angle);
  const double sin_theta = std::sin(angle);
  if (sl_point.s > 1e-4 && sl_point.s < length_ - 1e-4) {
    (*xy_point)[0] = matched_ref_point.x() - sin_theta * sl_point.l;
    (*xy_point)[1] = matched_ref_point.y() + cos_theta * sl_point.l;

  } else if (sl_point.s < 1e-4) {
    Eigen::Matrix2d mat;
    mat << cos_theta, sin_theta, -sin_theta, cos_theta;
    Eigen::Vector2d res;
    res << sl_point.s + cos_theta * matched_ref_point.x() + sin_theta * matched_ref_point.y(),
        sl_point.l + cos_theta * matched_ref_point.y() - sin_theta * matched_ref_point.x();
    Eigen::Vector2d xy = mat.reverse() * res;
    (*xy_point)[0] = xy.x();
    (*xy_point)[1] = xy.y();

  } else {
    Eigen::Matrix2d mat;
    mat << cos_theta, sin_theta, -sin_theta, cos_theta;
    Eigen::Vector2d res;
    res << sl_point.s - length_ + cos_theta * matched_ref_point.x() + sin_theta * matched_ref_point.y(),
        sl_point.l + cos_theta * matched_ref_point.y() - sin_theta * matched_ref_point.x();
    Eigen::Vector2d xy = mat.reverse() * res;
    (*xy_point)[0] = xy.x();
    (*xy_point)[1] = xy.y();
  }
  return true;
}

double ReferenceLine::DistanceToLineSegment(const Eigen::Vector2d &start,
                                            const Eigen::Vector2d &end,
                                            const Eigen::Vector2d &point) {
  double length = (end - start).norm();
  if (length < 1e-6) {
    return (point - start).norm();
  }
  Eigen::Vector2d unit_direction = (end - start);
  unit_direction.normalize();
  Eigen::Vector2d xy = point - start;
  const double proj = xy.dot(unit_direction);
  if (proj < 0.0) {
    return xy.norm();
  }

  if (proj > length) {
    return (end - point).norm();
  }
  return std::fabs(xy[0] * unit_direction[1] - xy[1] * unit_direction[0]);
}

bool ReferenceLine::BuildReferenceLineWithSpline() {
  std::vector<double> xs;
  xs.reserve(reference_points_.size());
  std::vector<double> ys;
  ys.reserve(reference_points_.size());
  for (const auto &ref_point : reference_points_) {
    xs.push_back(ref_point.x());
    ys.push_back(ref_point.y());
//    std::cout << "ref_point: x" << ref_point.x() << " , y : " << ref_point.y() << std::endl;

  }
  ref_line_spline_ = std::make_shared<Spline2d>(xs, ys);
//  std::cout << "ref_line_spline's length " << ref_line_spline_->ArcLength() << std::endl;
  std::vector<double> xs_left, ys_left;
  xs_left.reserve(left_boundary_.size());
  ys_left.reserve(left_boundary_.size());
  for (const auto &left : left_boundary_) {
    xs_left.push_back(left.x());
    ys_left.push_back(left.y());
//    std::cout << "left: x" << left.x() << " , y : " << left.y() << std::endl;
  }
  left_boundary_spline_ = std::make_shared<Spline2d>(xs_left, ys_left);

  std::vector<double> xs_right, ys_right;
  xs_right.reserve(right_boundary_.size());
  ys_right.reserve(right_boundary_.size());
  for (const auto &right : right_boundary_) {
    xs_right.push_back(right.x());
    ys_right.push_back(right.y());
//    std::cout << "right: x" << right.x() << " , y : " << right.y() << std::endl;

  }
  right_boundary_spline_ = std::make_shared<Spline2d>(xs_right, ys_right);
  return true;
}

bool ReferenceLine::Smooth(const double deviation_weight,
                           const double heading_weight,
                           const double distance_weight,
                           const double slack_weight,
                           const double max_curvature) {
  const auto way_points = way_points_;
  std::vector<ReferencePoint> ref_point;
  reference_smoother_->SetSmoothParams(deviation_weight, distance_weight, heading_weight, slack_weight, max_curvature);
  bool result = reference_smoother_->SmoothReferenceLine(reference_points_, &ref_point);
  if (reference_points_.size() != ref_point.size()) {
    return false;
  }
  if (!result) {
    smoothed_ = false;
    return false;
  }
  smoothed_ = true;
  std::vector<double> xs, ys;
  xs.reserve(ref_point.size());
  ys.reserve(ref_point.size());
  for (auto &reference_point : ref_point) {
    xs.push_back(reference_point.x());
    ys.push_back(reference_point.y());
  }
  ref_line_spline_.reset(new Spline2d(xs, ys));
  length_ = ref_line_spline_->ArcLength();
  return true;
}

double ReferenceLine::GetDrivingWidth(const SLBoundary &sl_boundary) const {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  GetLaneWidth(sl_boundary.start_s, &lane_left_width, &lane_right_width);
  double driving_width = std::max(lane_left_width - sl_boundary.end_l,
                                  lane_right_width + sl_boundary.start_l);
  driving_width = std::min(lane_left_width + lane_right_width, driving_width);
  return driving_width;
}

planning_msgs::WayPoint ReferenceLine::NearestWayPoint(double x, double y, size_t *index) const {
  size_t way_point_size = way_points_.size();
  double min_dist = std::numeric_limits<double>::infinity();
  size_t min_index = 0;
  for (size_t i = 0; i < way_point_size; ++i) {
    double dist = std::hypot(x - way_points_[i].pose.position.x, y - way_points_[i].pose.position.y);
    if (dist < min_dist) {
      min_dist = dist;
      min_index = i;
    }
  }
  *index = min_index;
  return way_points_[min_index];
}

planning_msgs::WayPoint ReferenceLine::NearestWayPoint(double s) const {
  auto reference_point = GetReferencePoint(s);
  size_t index;
  return NearestWayPoint(reference_point.x(), reference_point.y(), &index);
}

bool ReferenceLine::GetMatchedPoint(double x, double y, ReferencePoint *matched_ref_point, double *matched_s) const {
  ROS_ASSERT(!reference_points_.empty());
  double nearest_x, nearest_y, nearest_s;
  if (!ref_line_spline_->GetNearestPointOnSpline(x, y, &nearest_x, &nearest_y, &nearest_s)) {
    return false;
  }
  double ref_dx, ref_dy;
  double ref_ddx, ref_ddy;
  double ref_dddx, ref_dddy;
  ref_line_spline_->EvaluateFirstDerivative(nearest_s, &ref_dx, &ref_dy);
  ref_line_spline_->EvaluateSecondDerivative(nearest_s, &ref_ddx, &ref_ddy);
  ref_line_spline_->EvaluateThirdDerivative(nearest_s, &ref_dddx, &ref_dddy);
  double heading = MathUtils::NormalizeAngle(std::atan2(ref_dy, ref_dx));
  double kappa = MathUtils::CalcKappa(ref_dx, ref_dy, ref_ddx, ref_ddy);
  double dkappa = MathUtils::CalcDKappa(ref_dx, ref_dy, ref_ddx, ref_ddy, ref_dddx, ref_dddy);
  matched_ref_point->set_xy(nearest_x, nearest_y);
  matched_ref_point->set_theta(heading);
  matched_ref_point->set_kappa(kappa);
  matched_ref_point->set_dkappa(dkappa);
  *matched_s = nearest_s;
  return true;
}

ReferenceLine::ReferenceLine(const ReferenceLine &other) {
  smoothed_ = other.smoothed_;
  way_points_ = other.way_points_;
  reference_points_ = other.reference_points_;
  left_boundary_ = other.left_boundary_;
  right_boundary_ = other.right_boundary_;
  length_ = other.length_; // the total length of this reference line
  ref_line_spline_ = other.ref_line_spline_;
  left_boundary_spline_ = other.left_boundary_spline_;
  right_boundary_spline_ = other.right_boundary_spline_;
  reference_smoother_ = other.reference_smoother_;
  priority_ = other.priority_;
}

bool ReferenceLine::CanChangeLeft(double s) const {
  auto waypoint = NearestWayPoint(s);
  if (waypoint.lane_change.type == planning_msgs::LaneChangeType::LEFT) {
    return true;
  }
  if (waypoint.lane_change.type == planning_msgs::LaneChangeType::BOTH) {
    return true;
  }
  return false;
}

bool ReferenceLine::CanChangeRight(double s) const {
  auto waypoint = NearestWayPoint(s);
  if (waypoint.lane_change.type == planning_msgs::LaneChangeType::RIGHT) {
    return true;
  }
  if (waypoint.lane_change.type == planning_msgs::LaneChangeType::BOTH) {
    return true;
  }
  return false;
}

bool ReferenceLine::IsBlockedByBox(const Box2d &box, double ego_width, double buffer) const {
  common::SLBoundary sl_boundary;
  if (!this->GetSLBoundary(box, &sl_boundary)) {
    return true;
  }
  if (!IsOnLane(sl_boundary)) {
    return false;
  }
  double drive_width = GetDrivingWidth(sl_boundary);
  return ego_width + buffer > drive_width;
}

bool ReferenceLine::HasJunctionInFront(double x, double y, double distance_threshold) const {
  common::SLPoint sl_point;
  if (!this->XYToSL(x, y, &sl_point)) {
    return false;
  }
  for (double s = sl_point.s; s < sl_point.s + distance_threshold; s += 2.0) {
    auto way_point = this->NearestWayPoint(sl_point.s);
    if (way_point.is_junction) {
      return true;
    }
  }
  return false;
}
}