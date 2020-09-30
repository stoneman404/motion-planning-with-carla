#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_COMMON_INCLUDE_ST_DATA_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_COMMON_INCLUDE_ST_DATA_HPP_
#include <Eigen/Core>
#include <vector>
#include <ros/ros.h>
#include "math_utils.hpp"
namespace planning {
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
   *
   * @param point_pairs
   */
  explicit STBoundary(const std::vector<std::pair<STPoint, STPoint>> &point_pairs);
  ~STBoundary() = default;

  /**
   *
   * @param curr_time
   * @param s_upper
   * @param s_lower
   * @return
   */
  bool GetBoundarySRange(double curr_time, double *s_upper, double *s_lower) const;

  /**
   *
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

  const STPoint &upper_left_point() const;
  const STPoint &upper_right_point() const;
  const STPoint &lower_left_point() const;
  const STPoint &lower_right_point() const;

  void set_upper_left_point(const STPoint &st_point);
  void set_upper_right_point(const STPoint &st_point);
  void set_lower_left_point(const STPoint &st_point);
  void set_lower_right_point(const STPoint &st_point);

 private:
  size_t Prev(size_t i) const;
  size_t Next(size_t i) const;
  static bool GetIndexRange(const std::vector<STPoint> &points, double t,
                            size_t *left_index, size_t *right_index);
 private:
  int id_{};

  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;
  double length_ = 1.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  std::vector<STPoint> points_;
  STPoint lower_left_point_;
  STPoint lower_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;
  size_t num_points_ = 0;
  double is_convex_ = false;

  double area_ = 0.0;

};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_COMMON_INCLUDE_ST_DATA_HPP_
