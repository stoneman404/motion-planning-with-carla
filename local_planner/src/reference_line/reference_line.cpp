#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include "reference_line/reference_line.hpp"
#include "math_utils.hpp"

namespace planning {

ReferenceLine::ReferenceLine(const planning_srvs::RouteResponse &route_response) {
  ROS_ASSERT(route_response.route.size() >= 2);
  Eigen::Vector2d heading;
  lane_left_width_.reserve(route_response.route.size());
  lane_right_width_.reserve(route_response.route.size());
  reference_points_.reserve(route_response.route.size());
  for (const auto &waypoint : route_response.route) {
    lane_left_width_.push_back(waypoint.lane_width / 2.0);
    lane_right_width_.push_back(waypoint.lane_width / 2.0);
  }
}

ReferenceLine::ReferenceLine(const std::vector<planning_msgs::WayPoint> &waypoints) {
  ROS_ASSERT(waypoints.size() >= 2);
  for (const auto &waypoint : waypoints) {

  }
}

ReferenceLine::ReferenceLine(const ReferenceLine &other) {

}

FrenetFramePoint ReferenceLine::GetFrenetFramePoint(const Eigen::Vector2d &xy) const {
  return FrenetFramePoint();
}
ReferencePoint ReferenceLine::GetReferencePoint(double s) const {
  return ReferencePoint();
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
  return false;
}
double ReferenceLine::Length() const {
  return 0;
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
    if ((v0.x() * v1.y() - v0.y() * v1.x()) < 0.0){
      sl_boundary->boundary_points.push_back(sl_mid);
    }
  }

  for (const auto& sl_point : sl_boundary->boundary_points){
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

}