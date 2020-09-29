#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_ST_GRAPH_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_ST_GRAPH_HPP_
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
 private:
  void SetUp(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
             std::shared_ptr<ReferenceLine> ref_line);

  void SetUpStaticObstacle(std::shared_ptr<Obstacle> obstacle,
                           std::shared_ptr<ReferenceLine> ref_line);

  void SetUpDynamicObstacle(std::shared_ptr<Obstacle> obstacle,
                            std::shared_ptr<ReferenceLine> ref_line);

 private:
  std::pair<double, double> time_range_;
  std::pair<double, double> s_range_;
  std::shared_ptr<ReferenceLine> reference_line_;
  std::array<double, 3> init_d_;
  std::unordered_map<int, STBoundary> obstacle_st_map_;
  std::vector<STBoundary> obstacles_st_boundary_;
  std::vector<SLBoundary> obstacles_sl_boundary_;
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_ST_GRAPH_HPP_