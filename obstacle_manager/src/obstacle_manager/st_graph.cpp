#include <utility>
#include "obstacle_manager/st_graph.hpp"
#include "obstacle_manager/obstacle.hpp"

namespace {
constexpr double kDefaultLaneWidth = 3.5;
double kLeftWidth = kDefaultLaneWidth / 2.0;
double kRightWidth = kDefaultLaneWidth / 2.0;
}
namespace planning {
using namespace common;
STGraph::STGraph(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                 const ReferenceLine &reference_line,
                 double s_start,
                 double s_end,
                 double t_start,
                 double t_end,
                 const std::array<double, 3> &init_d,
                 double max_lookahead_time,
                 double delta_t)
    : max_lookahed_time_(max_lookahead_time),
      delta_t_(delta_t),
      time_range_({t_start, t_end}),
      s_range_({s_start, s_end}),
      reference_line_(reference_line),
      init_d_(init_d) {

  ROS_ASSERT(s_end >= s_start);
  ROS_ASSERT(t_end >= t_start);

  SetUp(obstacles, reference_line_);
}

void STGraph::SetUp(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                    ReferenceLine &ref_line) {
//  obstacles_sl_boundary_.clear();
  for (const auto &obstacle : obstacles) {
    if (!obstacle->IsStatic()) {
      SetUpStaticObstacle(obstacle, ref_line);
    } else {
      SetUpDynamicObstacle(obstacle, ref_line);
    }
  }

//  // for static obstacles
//  std::sort(obstacles_sl_boundary_.begin(), obstacles_sl_boundary_.end(),
//            [](const SLBoundary &sl0, const SLBoundary &sl1) {
//              return sl0.start_s < sl1.start_s;
//            });

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
  int obstacle_id = obstacle->Id();
//  ref_line.GetLaneWidth(sl_boundary.start_s, &kLeftWidth, &kRightWidth);
  if (sl_boundary.start_s > s_range_.second || sl_boundary.end_s < s_range_.first ||
      sl_boundary.start_l > kLeftWidth || sl_boundary.end_l < -kRightWidth) {
    ROS_DEBUG("[STGraph::SetUpStaticObstacle], obstacle[%i] is out of range. ", obstacle_id);
    return;
  }
  STBoundary st_boundary;
  if (!MakeSTBoundary(obstacle, ref_line, st_boundary)) {
    return;
  }
  if (st_map_.find(obstacle->Id()) == st_map_.end()) {
    st_map_.emplace(obstacle->Id(), st_boundary);
  } else {
    st_map_[obstacle->Id()] = st_boundary;
  }
//  obstacles_sl_boundary_.push_back(std::move(sl_boundary));
}

bool STGraph::MakeSTBoundary(const std::shared_ptr<Obstacle> &obstacle,
                             const ReferenceLine &ref_line,
                             STBoundary &st_boundary) const {

  double relative_time = time_range_.first;
  std::vector<std::pair<STPoint, STPoint>> st_points;
  while (relative_time < time_range_.second) {
    planning_msgs::TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
    Box2d box = obstacle->GetBoundingBoxAtPoint(point);
    SLBoundary sl_boundary;
    if (!ref_line.GetSLBoundary(box, &sl_boundary)) {
      return false;
    }
    if (sl_boundary.start_s > s_range_.second || sl_boundary.end_s < s_range_.first ||
        sl_boundary.start_l > kLeftWidth || sl_boundary.end_l < -kRightWidth) {
      relative_time += delta_t_;
      continue;
    }
    STPoint lower_st_point(sl_boundary.start_s, relative_time);
    STPoint upper_st_point(sl_boundary.end_s, relative_time);
    st_points.emplace_back(lower_st_point, upper_st_point);
    relative_time += delta_t_;
  }
  if (st_points.empty()) {
    return false;
  }
  st_boundary = STBoundary(st_points);
  return true;
}

void STGraph::SetUpDynamicObstacle(const std::shared_ptr<Obstacle> &obstacle,
                                   const ReferenceLine &ref_line) {
  common::STBoundary st_boundary;
  if (!MakeSTBoundary(obstacle, ref_line, st_boundary)) {
    return;
  }
  if (st_map_.find(obstacle->Id()) == st_map_.end()) {
    st_map_.emplace(obstacle->Id(), st_boundary);
  } else {
    st_map_[obstacle->Id()] = st_boundary;
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
    double s_upper, s_lower;
    if (!pt_obstacle.GetBoundarySRange(t, &s_upper, &s_lower)) {
      continue;
    }

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
  double relative_time = time_range_.first;
  while (relative_time < time_range_.second) {
    double s_lower, s_upper;
    STPoint st_point;
    if (!pt_obstacle.GetBoundarySRange(relative_time, &s_upper, &s_lower)) {
      relative_time += t_density;
      continue;
    }
    if (s_dist < 0.0) {
      st_point.set_s(s_lower + s_dist);
      st_point.set_t(relative_time);
    } else {
      st_point.set_s(s_upper + s_dist);
      st_point.set_t(relative_time);
    }
    pt_pairs.emplace_back(st_point);
    relative_time += t_density;
  }
  return pt_pairs;
//  double s0;
//  double s1;
//  double t0;
//  double t1;
//
//  if (s_dist > 0.0) {
//    s0 = pt_obstacle.upper_left_point().s();
//    s1 = pt_obstacle.upper_right_point().s();
//    t0 = pt_obstacle.upper_left_point().t();
//    t1 = pt_obstacle.upper_right_point().t();
//  } else {
//    s0 = pt_obstacle.lower_left_point().s();
//    s1 = pt_obstacle.lower_right_point().s();
//    t0 = pt_obstacle.lower_left_point().t();
//    t1 = pt_obstacle.lower_right_point().t();
//  }
//
//  double time_gap = t1 - t0;
//  ROS_ASSERT(time_gap > -1e-5);
//  time_gap = std::fabs(time_gap);
//
//  auto num_sections = static_cast<size_t>(time_gap / t_density + 1);
//  double t_interval = time_gap / static_cast<double>(num_sections);
//
//  for (size_t i = 0; i <= num_sections; ++i) {
//    double t = t_interval * static_cast<double>(i) + t0;
//    double s = MathUtils::lerp(s0, t0, s1, t1, t) + s_dist;
//
//    STPoint ptt;
//    ptt.set_t(t);
//    ptt.set_s(s);
//    pt_pairs.push_back(ptt);
//  }

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