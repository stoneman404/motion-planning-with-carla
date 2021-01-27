#include <utility>
#include "obstacle_manager/st_graph.hpp"
#include "obstacle_manager/obstacle.hpp"
namespace planning {
using namespace common;
STGraph::STGraph(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                 const ReferenceLine &reference_line,
                 double s_start,
                 double s_end,
                 double t_start,
                 double t_end,
                 const std::array<double, 3> &init_d, double max_lookahead_time, double delta_t)
    : max_lookahed_time_(max_lookahead_time), delta_t_(delta_t) {
  ROS_ASSERT(s_end >= s_start);
  ROS_ASSERT(t_end >= t_start);
  s_range_.first = s_start;
  s_range_.second = s_end;
  time_range_.first = t_start;
  time_range_.second = t_end;
  reference_line_ = reference_line;
  init_d_ = init_d;
  SetUp(obstacles, reference_line_);
}
void STGraph::SetUp(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                    ReferenceLine &ref_line) {
  for (const auto &obstacle : obstacles) {
    if (!obstacle->IsStatic()) {
      SetUpStaticObstacle(obstacle, ref_line);
    } else {
      SetUpDynamicObstacle(obstacle, ref_line);
    }
  }

  // for static obstacles
  std::sort(obstacles_sl_boundary_.begin(), obstacles_sl_boundary_.end(),
            [](const SLBoundary &sl0, const SLBoundary &sl1) {
              return sl0.start_s < sl1.start_s;
            });

  // for static and dynamic obstacles
  for (auto &obstacle_st : st_map_) {
    obstacles_st_boundary_.push_back(obstacle_st.second);
  }
}

void STGraph::SetUpStaticObstacle(const std::shared_ptr<Obstacle> &obstacle,
                                  const ReferenceLine &ref_line) {
  auto box = obstacle->BoundingBox();
  SLBoundary sl_boundary;
  if (!ref_line.GetSLBoundary(box, &sl_boundary)) {
    ROS_DEBUG("[STGraph::SetUpStaticObstacle] Failed to GetSLBoundary.");
    return;
  }
  constexpr double kDefaultLaneWidth = 3.5;
  double left_width = kDefaultLaneWidth;
  double right_width = kDefaultLaneWidth;
  int obstacle_id = obstacle->Id();
//  ref_line.GetLaneWidth(sl_boundary.start_s, &left_width, &right_width);
  if (sl_boundary.start_s > s_range_.second || sl_boundary.end_s < s_range_.first ||
      sl_boundary.start_l > left_width || sl_boundary.end_l < -right_width) {
    ROS_DEBUG("[STGraph::SetUpStaticObstacle], obstacle[%i] is out of range. ", obstacle_id);
    return;
  }

  st_map_[obstacle_id].set_id(obstacle_id);
  st_map_[obstacle_id].set_lower_left_point(SetSTPoint(sl_boundary.start_s, 0.0));
  st_map_[obstacle_id].set_lower_right_point(SetSTPoint(sl_boundary.start_s, max_lookahed_time_));
  st_map_[obstacle_id].set_upper_left_point(SetSTPoint(sl_boundary.end_s, 0.0));
  st_map_[obstacle_id].set_upper_right_point(SetSTPoint(sl_boundary.end_s, max_lookahed_time_));
  obstacles_sl_boundary_.push_back(std::move(sl_boundary));
}

void STGraph::SetUpDynamicObstacle(const std::shared_ptr<Obstacle> &obstacle,
                                   const ReferenceLine &ref_line) {

  double relative_time = time_range_.first;
  while (relative_time < time_range_.second) {
    planning_msgs::TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
    Box2d box = obstacle->GetBoundingBoxAtPoint(point);
    SLBoundary sl_boundary;
    if (!ref_line.GetSLBoundary(box, &sl_boundary)) {
      ROS_FATAL("[STGraph::SetUpDynamicObstacle]: failed to get sl_boundary");
      return;
    }
    constexpr double kDefaultLaneWidth = 3.5;

    double left_width = kDefaultLaneWidth;
    double right_width = kDefaultLaneWidth;
//    ref_line.GetLaneWidth(sl_boundary.start_s, &left_width, &right_width);

    // The obstacle is not shown on the region to be considered.
    if (sl_boundary.start_s > s_range_.second || sl_boundary.end_s < s_range_.first ||
        sl_boundary.start_l > left_width || sl_boundary.end_l < -right_width) {
      if (st_map_.find(obstacle->Id()) != st_map_.end()) {
        break;
      }
      relative_time += delta_t_;
      continue;
    }

    if (st_map_.find(obstacle->Id()) == st_map_.end()) {
      st_map_[obstacle->Id()].set_id(obstacle->Id());
      st_map_[obstacle->Id()].set_lower_left_point(SetSTPoint(sl_boundary.start_s, relative_time));
      st_map_[obstacle->Id()].set_upper_left_point(SetSTPoint(sl_boundary.end_s, relative_time));
    }

    st_map_[obstacle->Id()].set_lower_right_point(SetSTPoint(sl_boundary.start_s, relative_time));
    st_map_[obstacle->Id()].set_upper_right_point(SetSTPoint(sl_boundary.end_s, relative_time));
    relative_time += delta_t_;
  }
}
STPoint STGraph::SetSTPoint(double s, double t) {
  STPoint st_point(s, t);
  return st_point;
}
bool STGraph::GetSTObstacle(int id, STBoundary *st_boundary) {
  if (st_boundary == nullptr) {
    return false;
  }
  if (st_map_.find(id) == st_map_.end()) {
    return false;
  } else {
    *st_boundary = st_map_[id];
    return true;
  }

}
const std::vector<STBoundary> &STGraph::GetObstaclesSTBoundary() const { return obstacles_st_boundary_; }

std::vector<std::pair<double, double>> STGraph::GetPathBlockingIntervals(const double t) const {
  ROS_ASSERT(time_range_.first <= t && t <= time_range_.second);
  std::vector<std::pair<double, double>> intervals;
  for (const auto &pt_obstacle : obstacles_st_boundary_) {
    if (t > pt_obstacle.max_t() || t < pt_obstacle.min_t()) {
      continue;
    }
    double s_upper = MathUtils::lerp(pt_obstacle.upper_left_point().s(),
                                     pt_obstacle.upper_left_point().t(),
                                     pt_obstacle.upper_right_point().s(),
                                     pt_obstacle.upper_right_point().t(), t);
    double s_lower = MathUtils::lerp(pt_obstacle.lower_left_point().s(),
                                     pt_obstacle.lower_left_point().t(),
                                     pt_obstacle.lower_right_point().s(),
                                     pt_obstacle.lower_right_point().t(), t);
    intervals.emplace_back(s_lower, s_upper);
  }
  return intervals;
}

std::vector<STPoint> STGraph::GetObstacleSurroundingPoints(int obstacle_id, double s_dist, double t_density) const {
  ROS_ASSERT(t_density > 0.0);
  std::vector<STPoint> pt_pairs;
  if (st_map_.find(obstacle_id) == st_map_.end()) {
    return pt_pairs;
  }
  const auto &pt_obstacle = st_map_.at(obstacle_id);
  double s0;
  double s1;
  double t0;
  double t1;

  if (s_dist > 0.0) {
    s0 = pt_obstacle.upper_left_point().s();
    s1 = pt_obstacle.upper_right_point().s();
    t0 = pt_obstacle.upper_left_point().t();
    t1 = pt_obstacle.upper_right_point().t();
  } else {
    s0 = pt_obstacle.lower_left_point().s();
    s1 = pt_obstacle.lower_right_point().s();
    t0 = pt_obstacle.lower_left_point().t();
    t1 = pt_obstacle.lower_right_point().t();
  }

  double time_gap = t1 - t0;
  ROS_ASSERT(time_gap > -1e-5);
  time_gap = std::fabs(time_gap);

  auto num_sections = static_cast<size_t>(time_gap / t_density + 1);
  double t_interval = time_gap / static_cast<double>(num_sections);

  for (size_t i = 0; i <= num_sections; ++i) {
    double t = t_interval * static_cast<double>(i) + t0;
    double s = MathUtils::lerp(s0, t0, s1, t1, t) + s_dist;

    STPoint ptt;
    ptt.set_t(t);
    ptt.set_s(s);
    pt_pairs.push_back(ptt);
  }

  return pt_pairs;

}
bool STGraph::IsObstacleInGraph(int obstacle_id) {
  return st_map_.find(obstacle_id) != st_map_.end();
}

std::vector<std::vector<std::pair<double, double>>>
STGraph::GetPathBlockingIntervals(double start_time,
                                  double end_time,
                                  double resolution) const {
  std::vector<std::vector<std::pair<double, double>>> intervals;
  for (double t = start_time; t <= end_time; t += resolution) {
    intervals.push_back(GetPathBlockingIntervals(t));
  }
  return intervals;
}

}