#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include "reference_line/reference_line.hpp"
#include "math_utils.hpp"
#include "coordinate_transformer.hpp"

namespace planning {;

ReferenceLine::ReferenceLine(const planning_srvs::RouteResponse &route_response)
    : way_points_(route_response.route) {
  ROS_ASSERT(route_response.route.size() >= PlanningConfig::Instance().spline_order());
  reference_smoother_ = std::make_unique<ReferenceLineSmoother>();

  size_t waypoints_size = route_response.route.size();
  reference_points_.reserve(waypoints_size);
  left_boundary_.reserve(waypoints_size);
  right_boundary_.reserve(waypoints_size);
  for (size_t i = 0; i < route_response.route.size(); ++i) {
    const auto way_point = route_response.route[i];
    const double left_width = way_point.lane_width / 2.0;
    const double right_width = way_point.lane_width / 2.0;
    Eigen::Vector2d heading;
    if (i + 1 >= route_response.route.size()) {
      heading = Eigen::Vector2d(route_response.route[i].pose.position.x -
                                    route_response.route[i - 1].pose.position.x,
                                route_response.route[i].pose.position.y -
                                    route_response.route[i - 1].pose.position.y);
    } else {
      heading = Eigen::Vector2d(route_response.route[i + 1].pose.position.x -
                                    route_response.route[i].pose.position.x,
                                route_response.route[i + 1].pose.position.y -
                                    route_response.route[i].pose.position.y);
    }
    auto ref_point = ReferencePoint(way_point.pose.position.x, way_point.pose.position.y);
    ref_point.set_xy(way_point.pose.position.x, way_point.pose.position.y);
    ref_point.set_heading(
        MathUtil::NormalizeAngle(std::atan2(heading(1), heading(0))));
    reference_points_.push_back(ref_point);
    left_boundary_.emplace_back(
        ref_point.x() - left_width * std::sin(ref_point.heading()),
        ref_point.y() + left_width * std::cos(ref_point.heading()));
    right_boundary_.emplace_back(
        ref_point.x() + right_width * std::sin(ref_point.heading()),
        ref_point.y() + right_width * std::cos(ref_point.heading()));
  }
  ROS_ASSERT(reference_points_.size() == waypoints_size);
  ROS_ASSERT(left_boundary_.size() == waypoints_size);
  ROS_ASSERT(right_boundary_.size() == waypoints_size);
  bool result = BuildReferenceLineWithSpline();
  ROS_ASSERT(result);
  length_ = ref_line_spline_->ArcLength();
}

ReferenceLine::ReferenceLine(const std::vector<planning_msgs::WayPoint> &waypoints)
    : way_points_(waypoints) {
  ROS_ASSERT(waypoints.size() >= PlanningConfig::Instance().spline_order());
  reference_smoother_ = std::make_unique<ReferenceLineSmoother>();
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
    ref_point.set_heading(MathUtil::NormalizeAngle(std::atan2(heading(1), heading(0))));
    reference_points_.push_back(ref_point);
    left_boundary_.emplace_back(
        ref_point.x() - left_width * std::sin(ref_point.heading()),
        ref_point.y() + left_width * std::cos(ref_point.heading()));
    right_boundary_.emplace_back(
        ref_point.x() + right_width * std::sin(ref_point.heading()),
        ref_point.y() + right_width * std::cos(ref_point.heading()));
  }

  ROS_ASSERT(reference_points_.size() == waypoints.size());
  ROS_ASSERT(left_boundary_.size() == waypoints.size());
  ROS_ASSERT(right_boundary_.size() == waypoints.size());
  bool result = BuildReferenceLineWithSpline();
  ROS_ASSERT(result);
  length_ = ref_line_spline_->ArcLength();

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
  const double theta_ref = ref_point.heading();
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
  double ref_heading = MathUtil::NormalizeAngle(std::atan2(ref_dy, ref_dx));
  double ref_kappa = MathUtil::CalcKappa(ref_dx, ref_dy, ref_ddx, ref_ddy);
  double ref_dkappa = MathUtil::CalcDKappa(ref_dx, ref_dy, ref_ddx, ref_ddy, ref_dddx, ref_dddy);
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
  double heading = MathUtil::NormalizeAngle(std::atan2(ref_dy, ref_dx));
  double kappa = MathUtil::CalcKappa(ref_dx, ref_dy, ref_ddx, ref_ddy);
  double dkappa = MathUtil::CalcDKappa(ref_dx, ref_dy, ref_ddx, ref_ddy, ref_dddx, ref_dddy);
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

  double start_s = std::numeric_limits<double>::max();
  double end_s = std::numeric_limits<double>::lowest();
  double start_l = std::numeric_limits<double>::max();
  double end_l = std::numeric_limits<double>::lowest();
  std::vector<Eigen::Vector2d> corners = box.GetAllCorners();
  std::vector<SLPoint> sl_corners;
  for (const auto &point : corners) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      ROS_ERROR("Failed to get projection for point : %lf, %lf", point[0], point[1]);
      return false;
    }
    sl_corners.push_back(sl_point);
  }

  for (size_t i = 0; i < corners.size(); ++i) {
    auto index0 = i;
    // 多边形必须闭合
    auto index1 = (i + 1) % corners.size();
    const auto &p0 = corners[index0];
    const auto &p1 = corners[index1];
    const auto &p_mid = (p0 + p1) * 0.5;
    SLPoint sl_mid;
    if (!XYToSL(p_mid, &sl_mid)) {
      ROS_ERROR("Failed to get projection for point : %lf, %lf", p_mid[0], p_mid[1]);
      return false;
    }
    Eigen::Vector2d v0 = Eigen::Vector2d(sl_corners[index1].s - sl_corners[index0].s,
                                         sl_corners[index1].l - sl_corners[index0].l);
    Eigen::Vector2d v1 = Eigen::Vector2d(sl_mid.s - sl_corners[index0].s,
                                         sl_mid.l - sl_corners[index0].l);
    sl_boundary->boundary_points.push_back(sl_corners[index0]);
    // corners 的顺序是从右前方逆时针排列的．所以当叉乘为负的时候，说明中点出了凸包
    if ((v0.x() * v1.y() - v0.y() * v1.x()) < 0.0) {
      sl_boundary->boundary_points.push_back(sl_mid);
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
  double x = MathUtil::lerp(p0.x(), s0, p1.x(), s1, s);
  double y = MathUtil::lerp(p0.y(), s0, p1.y(), s1, s);
  double heading = MathUtil::slerp(p0.heading(), s0, p1.heading(), s1, s);
  double kappa = MathUtil::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  double dkappa = MathUtil::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);
  auto ref_point = ReferencePoint(x, y, heading, kappa, dkappa);

  return ref_point;
}

bool ReferenceLine::XYToSL(const Eigen::Vector2d &xy, SLPoint *sl_point) const {
  double nearest_x, nearest_y, nearest_s;
  if (!ref_line_spline_->GetNearestPointOnSpline(
      xy(0), xy(1), &nearest_x, &nearest_y, &nearest_s)) {
    return false;
  }
  double x_der, y_der;
  Eigen::Vector2d heading;
  ref_line_spline_->EvaluateFirstDerivative(nearest_s, &x_der, &y_der);
  heading << x_der, y_der;
  heading.normalize();
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
  const auto angle = matched_ref_point.heading();
  (*xy_point)[0] = matched_ref_point.x() - std::sin(angle) * sl_point.l;
  (*xy_point)[1] = matched_ref_point.y() + std::cos(angle) * sl_point.l;
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
  }
  ref_line_spline_ = std::make_shared<Spline2d>(xs, ys, PlanningConfig::Instance().spline_order());
  std::vector<double> xs_left, ys_left;
  xs_left.reserve(left_boundary_.size());
  ys_left.reserve(left_boundary_.size());
  for (const auto &left : left_boundary_) {
    xs_left.push_back(left.x());
    ys_left.push_back(left.y());
  }
  left_boundary_spline_ = std::make_shared<Spline2d>(xs_left, ys_left, PlanningConfig::Instance().spline_order());

  std::vector<double> xs_right, ys_right;
  xs_right.reserve(right_boundary_.size());
  ys_right.reserve(right_boundary_.size());
  for (const auto &right : right_boundary_) {
    xs_right.push_back(right.x());
    ys_right.push_back(right.y());
  }
  right_boundary_spline_ = std::make_shared<Spline2d>(xs_right, ys_right, PlanningConfig::Instance().spline_order());
  return true;
}

bool ReferenceLine::Smooth() {
  const auto way_points = way_points_;
  bool result = reference_smoother_->SmoothReferenceLine(way_points, &reference_points_);
  if (way_points.size() != reference_points_.size()) {
    return false;
  }
  if (!result) {
    smoothed_ = false;
    return false;
  }
  smoothed_ = true;
  std::vector<double> xs, ys;
  xs.reserve(reference_points_.size());
  ys.reserve(reference_points_.size());
  for (auto &reference_point : reference_points_) {
    xs.push_back(reference_point.x());
    ys.push_back(reference_point.y());
  }
  ref_line_spline_.reset(new Spline2d(xs, ys, 3));
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

}