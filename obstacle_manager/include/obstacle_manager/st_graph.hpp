#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_PATH_TIME_GRAPH_ST_GRAPH_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_PATH_TIME_GRAPH_ST_GRAPH_HPP_

#include <vector>
#include <array>
#include <ros/ros.h>
#include <unordered_map>
#include "obstacle_manager/obstacle.hpp"
//#include "obstacle_filter/obstacle.hpp"
#include "reference_line/reference_line.hpp"
#include "math/frenet_frame.hpp"
namespace planning {
class STGraph {
 public:
  STGraph() = default;
  ~STGraph() = default;

  STGraph(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
          std::shared_ptr<ReferenceLine> reference_line,
          double s_start, double s_end, double t_start, double t_end,
          const std::array<double, 3> &init_d,
          double max_lookahead_time, double delta_t);
  /**
   *
   * @return
   */
  const std::vector<common::STBoundary> &GetObstaclesSTBoundary() const;

  /**
   *
   * @param id
   * @param st_boundary
   * @return
   */
  bool GetSTObstacle(int id, common::STBoundary *st_boundary);

  /**
   *
   * @param t
   * @return
   */
  std::vector<std::pair<double, double>> GetPathBlockingIntervals(
      const double t) const;

  std::vector<std::vector<std::pair<double, double>>> GetPathBlockingIntervals(double start_time,
                                                                               double end_time,
                                                                               double resolution) const;

  /**
   *
   * @return
   */
  std::pair<double, double> get_s_range() const { return s_range_; }

  /**
   *
   * @return
   */
  std::pair<double, double> get_time_range() const { return time_range_; }

  /**
   *
   * @param obstacle_id
   * @param s_dist
   * @param t_density
   * @return
   */
  std::vector<common::STPoint> GetObstacleSurroundingPoints(int obstacle_id,
                                                    double s_dist,
                                                    double t_density) const;

  bool IsObstacleInGraph(int obstacle_id);

 private:
  void SetUp(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
             std::shared_ptr<ReferenceLine> ref_line);

  void SetUpStaticObstacle(std::shared_ptr<Obstacle> obstacle,
                           std::shared_ptr<ReferenceLine> ref_line);

  void SetUpDynamicObstacle(std::shared_ptr<Obstacle> obstacle,
                            std::shared_ptr<ReferenceLine> ref_line);
  static common::STPoint SetSTPoint(double s, double t);

 private:
  double max_lookahed_time_{};
  double delta_t_{};
  std::pair<double, double> time_range_;
  std::pair<double, double> s_range_;
  std::shared_ptr<ReferenceLine> reference_line_;
  std::array<double, 3> init_d_{};
  std::unordered_map<int, common::STBoundary> obstacle_st_map_;
  std::vector<common::STBoundary> obstacles_st_boundary_;
  std::vector<common::SLBoundary> obstacles_sl_boundary_;
};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_PATH_TIME_GRAPH_ST_GRAPH_HPP_