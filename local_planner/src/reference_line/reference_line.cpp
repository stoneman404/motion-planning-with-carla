#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include "reference_line/reference_line.hpp"
#include "math_utils.hpp"
#include "coordinate_transformer.hpp"
namespace planning {

ReferenceLine::ReferenceLine(const planning_srvs::RouteResponse &route_response) {
  ROS_ASSERT(route_response.route.size() >= 2);
  lane_left_width_.reserve(route_response.route.size());
  lane_right_width_.reserve(route_response.route.size());
  reference_points_.reserve(route_response.route.size());
  left_boundary_.reserve(route_response.route.size());
  right_boundary_.reserve(route_response.route.size());
  accumulated_s_.reserve(route_response.route.size());
  unit_directions_.reserve(route_response.route.size());
  double s = 0.0;
  for (size_t i = 0; i < route_response.route.size(); ++i) {
    const auto waypoint = route_response.route[i];
    const double left_width = waypoint.lane_width / 2.0;
    const double right_width = waypoint.lane_width / 2.0;
    lane_left_width_.push_back(left_width);
    lane_right_width_.push_back(right_width);
    Eigen::Vector2d heading;
    if (i + 1 >= route_response.route.size()) {
      accumulated_s_.push_back(s);
      heading = Eigen::Vector2d(route_response.route[i].pose.position.x -
                                    route_response.route[i - 1].pose.position.x,
                                route_response.route[i].pose.position.y -
                                    route_response.route[i - 1].pose.position.y);
    } else {
      heading = Eigen::Vector2d(route_response.route[i + 1].pose.position.x -
                                    route_response.route[i].pose.position.x,
                                route_response.route[i + 1].pose.position.y -
                                    route_response.route[i].pose.position.y);
      s += std::hypot(heading[0], heading[1]);
    }

    auto ref_point = ReferencePoint(waypoint.pose.position.x, waypoint.pose.position.y,
                                    tf::getYaw(waypoint.pose.orientation));
    ref_point.set_xy(waypoint.pose.position.x, waypoint.pose.position.y);
    ref_point.set_heading(NormalizeAngle(tf::getYaw(waypoint.pose.orientation)));
    reference_points_.push_back(ref_point);
    left_boundary_.emplace_back(ref_point.x() - left_width * std::sin(ref_point.heading()),
                                ref_point.y() + left_width * std::cos(ref_point.heading()));
    right_boundary_.emplace_back(ref_point.x() + right_width * std::sin(ref_point.heading()),
                                 ref_point.y() + right_width * std::cos(ref_point.heading()));
    heading.normalize();
    unit_directions_.push_back(heading);
  }
  length_ = s;
  smoothed_ = false;
  size_t waypoints_size = route_response.route.size();
  ROS_ASSERT(unit_directions_.size() == waypoints_size);
  ROS_ASSERT(accumulated_s_.size() == waypoints_size);
  ROS_ASSERT(reference_points_.size() == waypoints_size);
  ROS_ASSERT(left_boundary_.size() == waypoints_size);
  ROS_ASSERT(right_boundary_.size() == waypoints_size);
  ROS_ASSERT(lane_right_width_.size() == waypoints_size);
  ROS_ASSERT(lane_left_width_.size() == waypoints_size);

}

ReferenceLine::ReferenceLine(const std::vector<planning_msgs::WayPoint> &waypoints) {
  ROS_ASSERT(waypoints.size() >= 2);
  lane_left_width_.reserve(waypoints.size());
  lane_right_width_.reserve(waypoints.size());
  reference_points_.reserve(waypoints.size());
  left_boundary_.reserve(waypoints.size());
  right_boundary_.reserve(waypoints.size());
  accumulated_s_.reserve(waypoints.size());
  unit_directions_.reserve(waypoints.size());
  double s = 0.0;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto waypoint = waypoints[i];
    const double left_width = waypoint.lane_width / 2.0;
    const double right_width = waypoint.lane_width / 2.0;
    lane_left_width_.push_back(left_width);
    lane_right_width_.push_back(right_width);
    Eigen::Vector2d heading;
    if (i + 1 >= waypoints.size()) {
      accumulated_s_.push_back(s);
      heading = Eigen::Vector2d(waypoints[i].pose.position.x -
                                    waypoints[i - 1].pose.position.x,
                                waypoints[i].pose.position.y -
                                    waypoints[i - 1].pose.position.y);
    } else {
      heading = Eigen::Vector2d(waypoints[i + 1].pose.position.x -
                                    waypoints[i].pose.position.x,
                                waypoints[i + 1].pose.position.y -
                                    waypoints[i].pose.position.y);
      s += std::hypot(heading[0], heading[1]);
    }

    auto ref_point = ReferencePoint(waypoint.pose.position.x, waypoint.pose.position.y,
                                    tf::getYaw(waypoint.pose.orientation));
    ref_point.set_xy(waypoint.pose.position.x, waypoint.pose.position.y);
    ref_point.set_heading(NormalizeAngle(tf::getYaw(waypoint.pose.orientation)));
    reference_points_.push_back(ref_point);
    left_boundary_.emplace_back(ref_point.x() - left_width * std::sin(ref_point.heading()),
                                ref_point.y() + left_width * std::cos(ref_point.heading()));
    right_boundary_.emplace_back(ref_point.x() + right_width * std::sin(ref_point.heading()),
                                 ref_point.y() + right_width * std::cos(ref_point.heading()));
    heading.normalize();
    unit_directions_.push_back(heading);
  }
  length_ = s;
  smoothed_ = false;
  ROS_ASSERT(reference_points_.size() == waypoints.size());
  ROS_ASSERT(unit_directions_.size() == waypoints.size());
  ROS_ASSERT(accumulated_s_.size() == waypoints.size());
  ROS_ASSERT(left_boundary_.size() == waypoints.size());
  ROS_ASSERT(right_boundary_.size() == waypoints.size());
  ROS_ASSERT(lane_right_width_.size() == waypoints.size());
  ROS_ASSERT(lane_right_width_.size() == waypoints.size());
}

ReferenceLine::ReferenceLine(const ReferenceLine &other) {
  this->smoothed_ = other.smoothed_;
  this->speed_limits_ = other.speed_limits_;
  this->length_ = other.length_;
  this->right_boundary_ = other.right_boundary_;
  this->left_boundary_ = other.left_boundary_;
  this->reference_points_ = other.reference_points_;
  this->lane_right_width_ = other.lane_right_width_;
  this->lane_left_width_ = other.lane_left_width_;
  this->unit_directions_ = other.unit_directions_;
  this->accumulated_s_ = other.accumulated_s_;
  this->use_spline_curve_ = other.use_spline_curve_;
}

FrenetFramePoint ReferenceLine::GetFrenetFramePoint(const planning_msgs::PathPoint &path_point) const {
  if (reference_points_.empty()) {
    return FrenetFramePoint();
  }
  SLPoint sl;
  XYToSL(path_point.x, path_point.y, &sl);
  FrenetFramePoint frenet_frame_point;
  frenet_frame_point.s = sl.s;
  frenet_frame_point.l = sl.l;
  const double theta = path_point.theta;
  const double kappa = path_point.kappa;
  const double l = frenet_frame_point.l;
  ReferencePoint ref_point = GetReferencePoint(frenet_frame_point.s);
  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();
  const double dl = CoordinateTransformer::CalcLateralDerivative(theta_ref, theta, l, kappa_ref);
  const double ddl = CoordinateTransformer::CalcSecondLateralDerivative(theta_ref, theta,
                                                                        kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point.dl = dl;
  frenet_frame_point.ddl = ddl;
  return frenet_frame_point;
}

ReferencePoint ReferenceLine::GetReferencePoint(double s) const {
  if (!use_spline_curve_) {
    if (reference_points_.empty()) {
      return ReferencePoint();
    }
    const std::vector<double> &accumulated_s = accumulated_s_;
    if (s < accumulated_s.front() - 1e-2) {
      return reference_points_.front();
    }
    if (s > accumulated_s.back() + 1e-2) {
      return reference_points_.back();
    }
    size_t index = GetIndex(s);
    size_t next_index = index + 1;
    if (next_index >= reference_points_.size()) {
      next_index = reference_points_.size() - 1;
    }
    ReferencePoint p0 = reference_points_[index];
    ReferencePoint p1 = reference_points_[next_index];
    const double s0 = accumulated_s[index];
    const double s1 = accumulated_s[next_index];
    return Interpolate(p0, p1, s0, s1, s);
  } else {
    // todo
    return ReferencePoint();
  }

}

ReferencePoint ReferenceLine::GetReferencePoint(double x, double y) {
  return ReferencePoint();
}

ReferencePoint ReferenceLine::GetReferencePoint(const std::pair<double, double> &xy) const {
  return ReferencePoint();
}

double ReferenceLine::GetSpeedLimitFromS(double s) const {
  return 0;
}

bool ReferenceLine::GetLaneWidth(double s, double *const left_width, double *const right_width) const {
  if (reference_points_.empty()) {
    return false;
  }
  if (s >= accumulated_s_.back()) {
    *left_width = lane_left_width_.back();
    *right_width = lane_right_width_.back();
    return true;
  } else if (s <= accumulated_s_.front()) {
    *left_width = lane_left_width_.front();
    *right_width = lane_right_width_.front();
    return true;
  }
  size_t index = GetIndex(s);
  size_t next_index = index + 1;
  if (next_index >= accumulated_s_.size()) {
    next_index = accumulated_s_.size() - 1;
  }

  double s0 = accumulated_s_[index];
  double s1 = accumulated_s_[next_index];
  //note: lw-> left lane width, rw-> right lane width
  double lw0 = lane_left_width_[index];
  double lw1 = lane_left_width_[next_index];
  double rw0 = lane_right_width_[index];
  double rw1 = lane_right_width_[next_index];
  *left_width = lerp(lw0, s0, lw1, s1, s);
  *right_width = lerp(rw0, s0, rw1, s1, s);
  return true;
}

double ReferenceLine::Length() const {
  return length_;
}

void ReferenceLine::AddSpeedLimit(double start_s, double end_s, double speed_limit) {

}

bool ReferenceLine::IsOnLane(const SLBoundary &sl_boundary) const {

  // if the obstacle is out of range
  if (sl_boundary.end_s < 0 || sl_boundary.start_l > Length()) {
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
    // corners 的顺序是从右前方逆时针排列的．所以当叉乘为负的时候，说明中点除了凸包
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

bool ReferenceLine::Smooth() {
  if (reference_points_.size() <= 2) {
    ROS_WARN("[ReferenceLine::Smooth], the number of reference_points is less than 3");
    return false;
  }

}
size_t ReferenceLine::GetIndex(double s) const {

  if (s > accumulated_s_.back()) {
    return accumulated_s_.size() - 1;
  } else if (s < 1e-2) {
    return 0;
  }
  auto low_iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  size_t distance = std::distance(accumulated_s_.begin(), low_iter);
  if (distance < accumulated_s_.size()) {
    return distance - 1;
  }
  return accumulated_s_.size() - 2;
}
ReferencePoint ReferenceLine::Interpolate(const ReferencePoint &p0,
                                          const ReferencePoint &p1,
                                          double s0,
                                          double s1,
                                          double s) const {
  double x = lerp(p0.x(), s0, p1.x(), s1, s);
  double y = lerp(p0.y(), s0, p1.y(), s1, s);
  double heading = slerp(p0.heading(), s0, p1.heading(), s1, s);
  double kappa = lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  double dkappa = lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);
  auto ref_point = ReferencePoint(x, y, heading, kappa, dkappa);

  return ref_point;
}

}