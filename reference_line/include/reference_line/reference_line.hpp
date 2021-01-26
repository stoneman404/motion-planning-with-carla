#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
#include <planning_srvs/RoutePlanService.h>
#include "polygon/box2d.hpp"
#include <planning_msgs/PathPoint.h>
#include "curves/spline2d.hpp"
#include "reference_point.hpp"
#include "reference_line_smoother.hpp"
#include "math/frenet_frame.hpp"

namespace planning {

typedef planning_msgs::CarlaRoadOption RoadOption;

class ReferenceLine {
 public:
  ReferenceLine() = default;
  ~ReferenceLine() = default;

  /**
   * copy constructor
   * @param other
   */
  ReferenceLine(const ReferenceLine &other);
  /**
   * @brief construct the reference line from waypoint
   * @param waypoints
   */
  explicit ReferenceLine(const std::vector<planning_msgs::WayPoint> &waypoints);

  /**
   * constructor
   * @param route_response
   */
//  explicit ReferenceLine(const planning_srvs::RoutePlanServiceResponse &route_response);

  bool Empty() const {
    return reference_points_.empty();
  }

  void SetPriority(int priority) { this->priority_ = priority; }

  /**
   * @brief: get the reference line's priority
   * @return :the priority of this reference line
   */
  int GetPriority() const { return priority_; }

  /**
   * @brief: smooth the reference line
   * @return : true if smoothing the reference line is successful, false otherwise
   */
  bool Smooth(const double deviation_weight,
              const double heading_weight,
              const double distance_weight,
              const double max_curvature);

  /**
   * transform the xy to sl point
   * @param xy
   * @param sl_point
   * @return
   */
  bool XYToSL(const Eigen::Vector2d &xy, common::SLPoint *sl_point) const;

  /**
   *
   * @param x
   * @param y
   * @param sl_point
   * @return
   */
  bool XYToSL(double x, double y, common::SLPoint *sl_point) const;

  /**
   *
   * @param sl_point
   * @param xy_point
   * @return
   */
  bool SLToXY(const common::SLPoint &sl_point, Eigen::Vector2d *xy_point) const;

  /**
   * get frenet frame from reference line, covert xy in cart cord to frenet frame
   * @param xy : position in cart coordinate
   * @return
   */
  common::FrenetFramePoint GetFrenetFramePoint(const planning_msgs::PathPoint &path_point) const;

  /**
   * get reference point according s
   * @param s
   * @return
   */
  ReferencePoint GetReferencePoint(double s) const;

  /**
   * get the projection reference point in reference line
   * @param x
   * @param y
   * @return
   */
  ReferencePoint GetReferencePoint(double x, double y) const;

  bool GetMatchedPoint(double x, double y, ReferencePoint *matched_ref_point, double *matched_s) const;

  /**
   * @brief: get the reference point
   * @param xy
   * @return
   */
  ReferencePoint GetReferencePoint(const std::pair<double, double> &xy) const;

  /**
   * @brief: get the left lane width and right lane width
   * @param: s
   * @param: left_width
   * @param: right_width
   * @return: the left lane width and right lane width
   */
  bool GetLaneWidth(double s, double *left_width, double *right_width) const;

  /**
   * @brief get the total length of this reference line
   * @return
   */
  double Length() const;

  /**
   * @brief: check the object has influence on this lane
   * @param sl_boundary
   * @return
   */
  bool IsOnLane(const common::SLBoundary &sl_boundary) const;

  bool IsOnLane(const common::SLPoint &sl_point) const;

  double GetDrivingWidth(const common::SLBoundary &sl_boundary) const;

  bool IsBlockedByBox(const common::Box2d &box, double ego_width, double buffer) const;
  /**
   * @brief : build object sl boundary
   * @param box : the object's bounding box
   * @param sl_boundary : the output sl_boundary
   * @return : false if build sl_boundary failed, true otherwise
   */
  bool GetSLBoundary(const common::Box2d &box, common::SLBoundary *sl_boundary) const;

  /**
   * @brief : check the reference line is smoothed or not
   * @return : true if the reference line is smoothed, false otherwise
   */
  bool IsSmoothedReferenceLine() const { return smoothed_; }

  bool HasJunctionInFront(double x, double y, double distance_threshold) const;

//  bool ExtendReferenceLine(double length) ;
  /**
  *
  * @param p0
  * @param p1
  * @param s0
  * @param s1
  * @param s
  * @return
  */
  static ReferencePoint Interpolate(const ReferencePoint &p0,
                                    const ReferencePoint &p1,
                                    double s0,
                                    double s1,
                                    double s);
  const std::vector<planning_msgs::WayPoint> &way_points() const { return way_points_; }
  bool CanChangeLeft(double s) const;
  bool CanChangeRight(double s) const;

//  bool ExtendReferenceLine(double extend_length) {
//    if (extend_length < 1e-2) {
//      //do nothing
//      return true;
//    }
//    const double kGap = 2.0;
//    const auto kExtendNum = static_cast<size_t>(extend_length / kGap);
//    if (kExtendNum < 1) {
//      return true;
//    }
//    auto last_ref_point = reference_points_.back();
//    auto last_waypoint = ref_points_.back();
//    for (size_t i = 1; i < kExtendNum; ++i){
//      length_ += kGap;
//      double cos_theta = std::cos(last_ref_point.theta());
//      double sin_theta = std::sin(last_ref_point.theta());
//      ReferencePoint next_ref_point = last_ref_point;
//      next_ref_point.set_xy(last_ref_point.x() + kGap * cos_theta, last_ref_point.y() + kGap * sin_theta);
//      reference_points_.emplace_back(next_ref_point);
//      auto next_waypoint = last_waypoint;
//      next_waypoint.s += kGap;
//      next_waypoint.pose.position.x = last_waypoint.pose.position.x + kGap * cos_theta;
//      next_waypoint.pose.position.y = last_waypoint.pose.position.y + kGap * sin_theta;
//      next_waypoint.id = last_waypoint.id + 1;
//      next_waypoint.is_junction = false;
//      next_waypoint.has_left_lane = false;
//      next_waypoint.has_right_lane = false;
//      next_waypoint.lane_change.type = planning_msgs::LaneChangeType::FORWARD;
//      next_waypoint.
//    }
//
//
//  }

 private:
  planning_msgs::WayPoint NearestWayPoint(double x, double y, size_t *min_index) const;
  planning_msgs::WayPoint NearestWayPoint(double s) const;

  bool BuildReferenceLineWithSpline();
  /**
   *
   * @param start
   * @param end
   * @param point
   * @return
   */
  static inline double DistanceToLineSegment(const Eigen::Vector2d &start,
                                             const Eigen::Vector2d &end,
                                             const Eigen::Vector2d &point);

 private:
  bool smoothed_ = false;
  std::vector<planning_msgs::WayPoint> way_points_;
  std::vector<ReferencePoint> reference_points_;
  std::vector<Eigen::Vector2d> left_boundary_;
  std::vector<Eigen::Vector2d> right_boundary_;
  double length_{}; // the total length of this reference line
  std::shared_ptr<common::Spline2d> ref_line_spline_;
  std::shared_ptr<common::Spline2d> left_boundary_spline_;
  std::shared_ptr<common::Spline2d> right_boundary_spline_;
  std::shared_ptr<ReferenceLineSmoother> reference_smoother_;
  int priority_ = 0;
};

}
#endif //CATKIN_WS_SRC_PLANNING_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
