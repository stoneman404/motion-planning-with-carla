#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
#include "reference_point.hpp"
#include <planning_srvs/Route.h>
#include <box2d.hpp>
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
     * transform the xy to sl point
     * @param xy
     * @param sl_point
     * @return
     */
    bool XYToSL(const Eigen::Vector2d& xy, SLPoint* sl_point) const;
    /**
     *
     * @param x
     * @param y
     * @param sl_point
     * @return
     */
    bool XYToSL(double x, double y, SLPoint* sl_point) const;

    /**
     *
     * @param sl_point
     * @param xy_point
     * @return
     */
    bool SLToXY(const SLPoint& sl_point, Eigen::Vector2d* xy_point) const;



    /**
     * constructor
     * @param route_response
     */
    explicit ReferenceLine(const planning_srvs::RouteResponse &route_response);

    /**
     * get frenet frame from reference line, covert xy in cart cord to frenet frame
     * @param xy : position in cart coordinate
     * @return
     */
    FrenetFramePoint GetFrenetFramePoint(const Eigen::Vector2d &xy) const;

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
    ReferencePoint GetReferencePoint(double x, double y);

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

    /**
     * @brief: get the speed limit from s
     * @param s
     * @return
     */
    double GetSpeedLimitFromS(double s) const;

    /**
     * @brief: add speed limit
     * @param start_s
     * @param end_s
     * @param speed_limit
     */
    void AddSpeedLimit(double start_s, double end_s, double speed_limit);

    /**
     * @brief: check the obstacle is on lane
     * @param sl_boundary
     * @return
     */
    bool IsOnLane(const SLBoundary& sl_boundary) const;

    bool GetSLBoundary(const Box2d& box, SLBoundary* sl_boundary) const;

private:
    std::vector<ReferencePoint> reference_points_;
    std::vector<SpeedLimit> speed_limits_;
    std::vector<double> lane_left_width_;
    std::vector<double> lane_right_width_;
    double length_; // the total length of this reference line
    planning_msgs::CarlaRoadOption road_option_;

};

}
#endif //CATKIN_WS_SRC_PLANNING_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
