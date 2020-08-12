#ifndef CATKIN_WS_SRC_PLANNING_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
#define CATKIN_WS_SRC_PLANNING_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
#include "reference_point.hpp"
#include <planning_srvs/Route.h>
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
     * constructor
     * @param reference_points
     */
    explicit ReferenceLine(const std::vector<ReferencePoint> &reference_points);
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
    FrenetFramePoint GetFrenetFramePoint(const Eigen::Vector2d& xy) const;
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
    ReferencePoint GetReferencePoint(const std::pair<double, double>& xy) const;
    bool GetLaneWidth(double s, double* const left_width, double* const right_width) const;
    double Length() const;
    double GetSpeedLimitFromS(double s) const;
    void AddSpeedLimit(double start_s, double end_s, double speed_limit);


private:
    std::vector<ReferencePoint> reference_point_;
    std::vector<SpeedLimit> speed_limits_;
    double length_; // the total length of this reference line


};

}
#endif //CATKIN_WS_SRC_PLANNING_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_HPP_
