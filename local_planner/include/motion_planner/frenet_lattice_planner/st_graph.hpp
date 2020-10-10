#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MOTION_PLANNER_FRENET_LATTICE_PLANNER_ST_GRAPH_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MOTION_PLANNER_FRENET_LATTICE_PLANNER_ST_GRAPH_HPP_

#include <vector>
#include <array>
#include <ros/ros.h>
#include <unordered_map>
#include "obstacle_filter/obstacle.hpp"
#include "reference_line/reference_line.hpp"
#include "st_data.hpp"
namespace planning {
class STGraph {
 public:
  STGraph() = default;
  ~STGraph() = default;

  STGraph(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
          std::shared_ptr<ReferenceLine> reference_line,
          double s_start, double s_end, double t_start, double t_end,
          const std::array<double, 3> &init_d);
  /**
   *
   * @return
   */
  const std::vector<STBoundary> &GetObstaclesSTBoundary() const;

  /**
   *
   * @param id
   * @param st_boundary
   * @return
   */
  bool GetSTObstacle(int id, STBoundary *st_boundary);

  /**
   *
   * @param t
   * @return
   */
  std::vector<std::pair<double, double>> GetPathBlockingIntervals(
      const double t) const;

  /**
   *
   * @param t_start
   * @param t_end
   * @param t_resolution
   * @return
   */
  std::vector<std::vector<std::pair<double, double>>> GetPathBlockingInterval(const double t_start,
                                                                              const double t_end,
                                                                              const double t_resolution) const;

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
  std::vector<STPoint> GetObstacleSurroundingPoints(int obstacle_id,
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
  static STPoint SetSTPoint(double s, double t);

 private:
  std::pair<double, double> time_range_;
  std::pair<double, double> s_range_;
  std::shared_ptr<ReferenceLine> reference_line_;
  std::array<double, 3> init_d_{};
  std::unordered_map<int, STBoundary> obstacle_st_map_;
  std::vector<STBoundary> obstacles_st_boundary_;
  std::vector<SLBoundary> obstacles_sl_boundary_;
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_ST_GRAPH_HPP_