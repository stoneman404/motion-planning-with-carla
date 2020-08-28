#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
#include <planning_srvs/Route.h>
#include <box2d.hpp>
#include <planning_msgs/PathPoint.h>
#include "spline2d.hpp"
#include "reference_point.hpp"

namespace planning {

struct SpeedLimit {
  SpeedLimit() = default;
  SpeedLimit(double _start_s, double _end_s, double _speed_limit)
      : start_s(_speed_limit), end_s(_end_s), speed_limit(_speed_limit) {}
  double start_s = 0.0;
  double end_s = 0.0;
  double speed_limit = 0.0;
};

struct FrenetFramePoint {
  FrenetFramePoint() = default;
  ~FrenetFramePoint() = default;
  FrenetFramePoint(double _s, double _l, double _dl, double _ddl)
      : s(_s), l(_l), dl(_dl), ddl(_ddl) {}
  double s = 0.0;
  double l = 0.0;
  double dl = 0.0;
  double ddl = 0.0;
};

struct SLPoint {
  SLPoint() = default;
  ~SLPoint() = default;
  SLPoint(double _s, double _l) : s(_s), l(_l) {}
  double s = 0.0;
  double l = 0.0;
};

struct SLBoundary {

  SLBoundary() = default;
  SLBoundary(double _start_s, double _end_s,
             double _start_l, double _end_l)
      : start_s(_start_s), end_s(_end_s),
        start_l(_start_l), end_l(_end_l) {}

  double start_s = 0.0;
  double end_s = 0.0;
  double start_l = 0.0;
  double end_l = 0.0;
  std::vector<SLPoint> boundary_points = std::vector<SLPoint>();
};

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
    explicit ReferenceLine(const planning_srvs::RouteResponse &route_response);

    /**
     * @brief: update the reference points, which will has
     * influence on accumulated s and unit directions
     * @param ref_points
     * @return
     */
    bool UpdateReferencePoints(const std::vector<ReferencePoint> &ref_points);

    /**
     * transform the xy to sl point
     * @param xy
     * @param sl_point
     * @return
     */
    bool XYToSL(const Eigen::Vector2d &xy, SLPoint *sl_point) const;

    /**
     *
     * @param x
     * @param y
     * @param sl_point
     * @return
     */
    bool XYToSL(double x, double y, SLPoint *sl_point) const;

    /**
     *
     * @param sl_point
     * @param xy_point
     * @return
     */
    bool SLToXY(const SLPoint &sl_point, Eigen::Vector2d *xy_point) const;

    /**
     * get frenet frame from reference line, covert xy in cart cord to frenet frame
     * @param xy : position in cart coordinate
     * @return
     */
    FrenetFramePoint GetFrenetFramePoint(const planning_msgs::PathPoint &path_point) const;

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

    /**
     * @brief: get the reference point
     * @param xy
     * @return
     */
    ReferencePoint GetReferencePoint(const std::pair<double, double> &xy) const;

    /**
     * get the left lane width and right lane width
     * @param s
     * @param left_width
     * @param right_width
     * @return
     */
    bool GetLaneWidth(double s, double *left_width, double *right_width) const;

    /**
     * @brief get the total length of this reference line
     * @return
     */
    double Length() const;

    ///////////////////////////////////////////////////////////////////////////
    /**
     * @brief: check the object has influence on this lane
     * @param sl_boundary
     * @return
     */
    bool IsOnLane(const SLBoundary &sl_boundary) const;

    /**
     * @brief : build object sl boundary
     * @param box : the object's bounding box
     * @param sl_boundary : the output sl_boundary
     * @return : false if build sl_boundary failed, true otherwise
     */
    bool GetSLBoundary(const Box2d &box, SLBoundary *sl_boundary) const;

    /**
     * @brief : check the reference line is smoothed or not
     * @return : true if the reference line is smoothed, false otherwise
     */
    bool IsSmoothedReferenceLine() const { return smoothed_; }

    /**
    *
    * @param p0
    * @param p1
    * @param s0
    * @param s1
    * @param s
    * @return
    */
    ReferencePoint Interpolate(const ReferencePoint &p0,
                               const ReferencePoint &p1,
                               double s0,
                               double s1,
                               double s) const;

    bool RefineReferenceLine();

    const std::vector<double> &GetAccumulatedS() const { return accumulated_s_; }
    std::vector<ReferencePoint> &mutable_reference_points() { return reference_points_; }
    const std::vector<ReferencePoint> &reference_points() const { return reference_points_; }
    const std::vector<Eigen::Vector2d> &right_boundary() const { return right_boundary_; }
    const std::vector<Eigen::Vector2d> &left_boundary() const { return left_boundary_; }
    // first is left boundary, second is right boundary
    std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> boundary() const {
      const auto right_boundary = right_boundary_;
      const auto left_boundary = left_boundary_;
      auto boundary_pair = std::make_pair(left_boundary, right_boundary);
      return boundary_pair;
    }
    const std::vector<double> &right_lane_width() const { return lane_right_width_; }
    const std::vector<double> &left_lane_width() const { return lane_left_width_; }

private:

    bool BuildReferenceLineWithSpline();
    /**
     *
     * @param start
     * @param end
     * @param point
     * @return
     */
    inline double DistanceToLineSegment(const Eigen::Vector2d &start,
                                        const Eigen::Vector2d &end,
                                        const Eigen::Vector2d &point) const;

    size_t GetIndex(double s) const;


private:
    bool smoothed_ = false;
    std::vector<ReferencePoint> reference_points_;
//    std::vector<SpeedLimit> speed_limits_;
    std::vector<double> lane_left_width_;
    std::vector<double> lane_right_width_;
    std::vector<Eigen::Vector2d> left_boundary_;
    std::vector<Eigen::Vector2d> right_boundary_;
    std::vector<double> accumulated_s_;
    std::vector<Eigen::Vector2d> unit_directions_;
    double length_; // the total length of this reference line
    bool use_spline_curve_ = true;
    std::shared_ptr<Spline2d> ref_line_spline_ = nullptr;
    std::shared_ptr<Spline2d> left_boundary_spline_ = nullptr;
    std::shared_ptr<Spline2d> right_boundary_spline_ = nullptr;
};

}
#endif //CATKIN_WS_SRC_PLANNING_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
