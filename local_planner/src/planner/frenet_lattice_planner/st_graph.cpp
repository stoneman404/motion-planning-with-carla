#include "planner/frenet_lattice_planner/st_graph.hpp"

#include <utility>
namespace planning {

STGraph::STGraph(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                 std::shared_ptr<ReferenceLine> reference_line,
                 double s_start,
                 double s_end,
                 double t_start,
                 double t_end,
                 const std::array<double, 3> &init_d) {
  ROS_ASSERT(s_end >= s_start);
  ROS_ASSERT(t_end >= t_start);
  s_range_.first = s_start;
  s_range_.second = s_end;
  time_range_.first = t_start;
  time_range_.second = t_end;
  reference_line_ = std::move(reference_line);
  init_d_ = init_d;
  SetUp(obstacles, reference_line_);
}
void STGraph::SetUp(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                    std::shared_ptr<ReferenceLine> ref_line) {
  for (const auto &obstacle : obstacles) {
    if (!obstacle->HasTrajectory()) {

    }
  }
}
void STGraph::SetUpStaticObstacle(std::shared_ptr<Obstacle> obstacle,
                                  std::shared_ptr<ReferenceLine> ref_line) {
  auto box = obstacle->BoundingBox();
  SLBoundary sl_boundary;
  if (!ref_line->GetSLBoundary(box, &sl_boundary)) {
    ROS_DEBUG("[STGraph::SetUpStaticObstacle] Failed to GetSLBoundary.");
    return;
  }
  double left_width;
  double right_width;
  int obstacle_id = obstacle->Id();
  ref_line->GetLaneWidth(sl_boundary.start_s, &left_width, &right_width);
  if (sl_boundary.start_s > s_range_.second ||
      sl_boundary.end_s < s_range_.first ||
      sl_boundary.start_l > left_width ||
      sl_boundary.end_l < -right_width) {
    ROS_DEBUG("[STGraph::SetUpStaticObstacle], obstacle[%i] is out of range. ", obstacle_id);
    return;
  }
  if (obstacle_st_map_.find(obstacle_id) == obstacle_st_map_.end()) {
    std::vector<std::pair<STPoint, STPoint>> st_points;
    st_points.reserve(2);
    st_points.emplace_back();
    auto &st_pair_left = st_points.back();
    st_pair_left.first = STPoint(sl_boundary.start_s, 0.0);
    st_pair_left.second = STPoint(sl_boundary.end_s, 0.0);
    st_points.emplace_back();
    auto &st_pair_right = st_points.back();
    st_pair_right.first = STPoint(sl_boundary.start_s, PlanningConfig::Instance().max_lookahead_time());
    st_pair_right.second = STPoint(sl_boundary.end_s, PlanningConfig::Instance().max_lookahead_time());
    obstacle_st_map_.emplace(obstacle_id, STBoundary(st_points));
    obstacle_st_map_[obstacle_id].set_id(obstacle_id);
    obstacles_sl_boundary_.emplace_back(sl_boundary);
  }

}

}