#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_PATH_TIME_GRAPH_FRENET_FRAME_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_PATH_TIME_GRAPH_FRENET_FRAME_HPP_
#include <Eigen/Core>
#include <vector>
#include <ros/ros.h>
#include "math/math_utils.hpp"
namespace common {

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
  SLPoint(const SLPoint &other) {
    s = other.s;
    l = other.l;
  }
  ~SLPoint() = default;
  SLPoint(double _s, double _l) : s(_s), l(_l) {}
  double s = 0.0;
  double l = 0.0;
};

struct SLBoundary {

  SLBoundary() = default;
  SLBoundary(const SLBoundary &other) {
    start_s = other.start_s;
    end_s = other.end_s;
    start_l = other.start_l;
    end_l = other.end_l;
    boundary_points = other.boundary_points;
  }
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



class STPoint {
 public:
  STPoint() = default;
  STPoint(double s, double t);
  explicit STPoint(const Eigen::Vector2d &st_point);
  double s() const;
  double t() const;
  void set_s(double s);
  void set_t(double t);
 private:
  double s_{};
  double t_{};
};

class STBoundary {
 public:
  STBoundary() = default;
  /**
   * @param point_pairs, first is lower st point, second is upper point
   */
  explicit STBoundary(const std::vector<std::pair<STPoint, STPoint>> &point_pairs);
  ~STBoundary() = default;

  /**
   * @param curr_time
   * @param s_upper
   * @param s_lower
   * @return
   */
  bool GetBoundarySRange(double curr_time, double *s_upper, double *s_lower) const;

  /**
   * @param curr_time
   * @param ds_upper
   * @param ds_lower
   * @return
   */
  bool GetBoundarySlopes(double curr_time, double *ds_upper, double *ds_lower) const;

  /**
   *
   * @param st_point
   * @return
   */
  bool IsPointInBoundary(const STPoint &st_point) const;
  int id() const;

  void set_id(int id);
  double min_s() const;
  double min_t() const;
  double max_s() const;
  double max_t() const;
  const std::vector<STPoint> &upper_points() const;
  const std::vector<STPoint> &lower_points() const;
 private:
  size_t Prev(size_t i) const;
  size_t Next(size_t i) const;
  static bool GetIndexRange(const std::vector<STPoint> &points, double t,
                            size_t *left_index, size_t *right_index);
 private:
  int id_{};
  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();
  std::vector<STPoint> points_;
  size_t num_points_ = 0;
};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_COMMON_INCLUDE_ST_DATA_HPP_
