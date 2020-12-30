#include <tf/transform_datatypes.h>
#include "behaviour.hpp"
namespace planning {

void ProbDistributeOfLatBehaviour::SetEntry(const LateralBehaviour &behaviour,
                                            double prob,
                                            const ReferenceLine &lane) {

  prob_dist_queue_.emplace(std::make_pair(behaviour, lane), prob);
}

bool ProbDistributeOfLatBehaviour::GetMaxProbBehaviour(LateralBehaviour &behaviour) const {
  if (prob_dist_queue_.empty()) {
    return false;
  }
  behaviour = prob_dist_queue_.top().first.first;
  return true;
}

bool ProbDistributeOfLatBehaviour::GetMaxProbBehaviourAndLane(LateralBehaviour &behaviour,
                                                              ReferenceLine &lane) const {
  if (prob_dist_queue_.empty()) {
    return false;
  }
  behaviour = prob_dist_queue_.top().first.first;
  lane = prob_dist_queue_.top().first.second;
  return true;
}

bool ProbDistributeOfLatBehaviour::GetKthMaxProbBehavioursAndLanes(
    uint32_t k,
    std::vector<std::pair<LateralBehaviour, ReferenceLine>> &behaviour_lane_pairs) {
  if (k < 1) {
    return false;
  }
  if (prob_dist_queue_.empty()) {
    return false;
  }
  behaviour_lane_pairs.clear();
  uint32_t cnt = 0;
  while (!prob_dist_queue_.empty() && cnt < k) {
    auto behaviour_pair = prob_dist_queue_.top();
    behaviour_lane_pairs.emplace_back(behaviour_pair.first);
    prob_dist_queue_.pop();
    ++cnt;
  }
  return true;
}

/********************************** ReferenceInfo ******************************/
ReferenceInfo::ReferenceInfo(const ReferenceLineConfig &config, double lookahead_distance, double lookback_distance)
    : smooth_config_(config),
      lookahead_distance_(lookahead_distance),
      lookback_distance_(lookback_distance),
      reference_line_history_(boost::circular_buffer<std::vector<ReferenceLine>>(3)) {
  is_initialized_ = true;
}

bool ReferenceInfo::Start() {
  if (!is_initialized_) {
    return false;
  }
  is_stop_ = false;
  task_future_ = std::async(std::launch::async, [this] { GenerateThread(); });
  return true;
}

bool ReferenceInfo::CreateReferenceLines(bool smooth, std::vector<ReferenceLine> &ref_lanes) {
  if (!has_route_ || !has_vehicle_state_) {
    return false;
  }
  vehicle_state::KinoDynamicState vehicle_state{};
  {
    std::lock_guard<std::mutex> lock_guard(vehicle_mutex_);
    vehicle_state = vehicle_state_;
  }

  RouteInfo route_info;
  {
    std::lock_guard<std::mutex> lock_guard(route_mutex_);
    route_info = route_info_;
  }

  auto main_ref_lane = ReferenceLine();
  auto result = ReferenceInfo::RetriveReferenceLine(
      main_ref_lane, vehicle_state,
      route_info.main_lane,
      lookahead_distance_,
      lookback_distance_,
      smooth, smooth_config_);
  if (!result) {
    return false;
  }
  ref_lanes.emplace_back(main_ref_lane);
  bool has_overlap_left_lane = false;
  size_t overlap_left_index = 0;
  for (size_t i = 0; i < route_info.left_lanes.size(); ++i) {
    auto left_lane = route_info.left_lanes[i];
    double overlap_begin_s, overlap_end_s;
    if (HasOverLapWithRefLane(main_ref_lane, left_lane, &overlap_begin_s, &overlap_end_s)) {
      overlap_left_index = i;
      has_overlap_left_lane = true;
      break;
    }
  }
  if (has_overlap_left_lane) {
    auto left_ref_lane = ReferenceLine();
    if (RetriveReferenceLine(
        left_ref_lane, vehicle_state,
        route_info.left_lanes[overlap_left_index],
        lookback_distance_,
        lookback_distance_,
        smooth, smooth_config_)) {
      ref_lanes.emplace_back(left_ref_lane);
    }
  }

  bool has_overlap_right_lane = false;
  size_t overlap_right_index = 0;
  for (size_t i = 0; i < route_info.right_lanes.size(); ++i) {
    auto right_lane = route_info.right_lanes[i];
    double overlap_begin_s, overlap_end_s;
    if (HasOverLapWithRefLane(main_ref_lane, right_lane, &overlap_begin_s, &overlap_end_s)) {
      overlap_right_index = i;
      has_overlap_right_lane = true;
      break;
    }
  }
  if (has_overlap_right_lane) {
    auto right_ref_lane = ReferenceLine();
    if (RetriveReferenceLine(
        right_ref_lane, vehicle_state,
        route_info.right_lanes[overlap_right_index],
        lookback_distance_,
        lookback_distance_,
        smooth, smooth_config_)) {
      ref_lanes.emplace_back(right_ref_lane);
    }
  }
  return true;
}

bool ReferenceInfo::RetriveReferenceLine(ReferenceLine &ref_lane,
                                         const vehicle_state::KinoDynamicState &vehicle_state,
                                         const std::vector<planning_msgs::WayPoint> &lane,
                                         double lookahead_distance,
                                         double lookback_distance,
                                         bool smooth,
                                         const ReferenceLineConfig &smooth_config) {
  auto dist_sqr = [](const planning_msgs::WayPoint &way_point, Eigen::Vector2d &xy) -> double {
    return (way_point.pose.position.x - xy.x()) * (way_point.pose.position.x - xy.x())
        + (way_point.pose.position.y - xy.y()) * (way_point.pose.position.y - xy.y());
  };
  size_t index_min = 0;
  Eigen::Vector2d xy{vehicle_state.x, vehicle_state.y};
  double d_min = dist_sqr(lane.front(), xy);
  for (size_t i = 1; i < lane.size(); ++i) {
    double d = dist_sqr(lane[i], xy);
    if (d < d_min) {
      d_min = d;
      index_min = i;
    }
  }
  std::vector<planning_msgs::WayPoint> sampled_way_points;
  double s = 0;
  size_t index = index_min == 0 ? index_min : index_min - 1;;
  while (index > 0 || s > lookback_distance - std::numeric_limits<double>::epsilon()) {
    sampled_way_points.push_back(lane[index]);
    Eigen::Vector2d xy_last{lane[index].pose.position.x, lane[index].pose.position.y};
    s += std::sqrt(dist_sqr(lane[--index], xy_last));
  }
  if (!sampled_way_points.empty()) {
    std::reverse(sampled_way_points.begin(), sampled_way_points.end());
  }
  index = index_min;
  s = 0;
  while (index < lane.size() || s > lookahead_distance - std::numeric_limits<double>::epsilon()) {
    sampled_way_points.push_back(lane[index]);
    Eigen::Vector2d xy_last{lane[index].pose.position.x, lane[index].pose.position.y};
    s += std::sqrt(dist_sqr(lane[++index], xy_last));
  }
  if (sampled_way_points.size() < 3) {
    return false;
  }
//  auto main_ref_lane = ReferenceLine(sampled_way_points);
  ref_lane = ReferenceLine(sampled_way_points);
  if (smooth) {
    if (!ref_lane.Smooth(smooth_config.reference_smooth_deviation_weight_,
                         smooth_config.reference_smooth_heading_weight_,
                         smooth_config.reference_smooth_length_weight_,
                         smooth_config.reference_smooth_max_curvature_)) {
      ROS_WARN("Failed to Smooth Reference Line");
    }
  }

  return true;
}

bool ReferenceInfo::HasOverLapWithRefLane(const ReferenceLine &ref_lane,
                                          std::vector<planning_msgs::WayPoint> &waypoints,
                                          double *overlap_start_s,
                                          double *overlap_end_s) {
  if (waypoints.size() < 3) {
    return false;
  }
  const auto begin = waypoints.front();
  const auto end = waypoints.back();
  common::SLPoint begin_sl, end_sl;
  constexpr double kMaxAngleDiff = 0.25 * M_PI;
  if (!ref_lane.XYToSL(begin.pose.position.x, begin.pose.position.y, &begin_sl)) {
    return false;
  }
  if (begin_sl.s > ref_lane.Length() - std::numeric_limits<double>::epsilon()) {
    return false;
  }
  auto begin_ref_point = ref_lane.GetReferencePoint(begin_sl.s);
  const double begin_angle_diff =
      common::MathUtils::CalcAngleDist(common::MathUtils::NormalizeAngle(tf::getYaw(begin.pose.orientation)),
                                       begin_ref_point.theta());
  if (std::fabs(begin_angle_diff) > kMaxAngleDiff) {
    return false;
  }
  if (!ref_lane.XYToSL(end.pose.position.x, end.pose.position.y, &end_sl)) {
    return false;
  }
  if (end_sl.s < std::numeric_limits<double>::epsilon()) {
    return false;
  }
  auto end_ref_point = ref_lane.GetReferencePoint(begin_sl.s);
  const double end_angle_diff =
      common::MathUtils::CalcAngleDist(common::MathUtils::NormalizeAngle(tf::getYaw(end.pose.orientation)),
                                       end_ref_point.theta());
  if (std::fabs(end_angle_diff) > kMaxAngleDiff) {
    return false;
  }
  *overlap_start_s = begin_sl.s < std::numeric_limits<double>::epsilon() ? 0.0 : begin_sl.s;
  *overlap_end_s =
      end_sl.s > ref_lane.Length() - std::numeric_limits<double>::epsilon() ? ref_lane.Length() : end_sl.s;
  return true;
}

bool ReferenceInfo::UpdateRouteResponse(const planning_srvs::RoutePlanServiceResponse &route_response) {
  std::lock_guard<std::mutex> lock_guard(route_mutex_);
  auto raw_ref_lane = route_response.route;
  auto raw_left_lane = route_response.left_lane;
  auto raw_right_lane = route_response.right_lane;
  route_info_.right_lanes = ReferenceInfo::SplitRawLane(raw_right_lane);
  route_info_.left_lanes = ReferenceInfo::SplitRawLane(raw_ref_lane);
  has_route_ = true;
  return true;
}

std::vector<std::vector<planning_msgs::WayPoint>> ReferenceInfo::SplitRawLane(const planning_msgs::Lane &raw_lane) {
  std::vector<std::vector<planning_msgs::WayPoint>> split_lanes;
  std::vector<std::vector<planning_msgs::WayPoint>> lanes;
  auto dist = [](const planning_msgs::WayPoint &p1, const planning_msgs::WayPoint &p2) -> double {
    return std::hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  };
  constexpr double kMaxDistanceGap = 3.0;
  if (raw_lane.way_points.size() > 3) {
    split_lanes.emplace_back();
    auto last_waypoint = raw_lane.way_points[0];
    split_lanes.back().emplace_back(last_waypoint);
    for (size_t i = 1; i < raw_lane.way_points.size(); ++i) {
      auto cur_waypoint = raw_lane.way_points[i];
      if (dist(cur_waypoint, last_waypoint) > kMaxDistanceGap) {
        split_lanes.emplace_back();
      }
      split_lanes.back().emplace_back(raw_lane.way_points[i]);
      last_waypoint = cur_waypoint;
    }
    for (const auto &lane : split_lanes) {
      if (lane.size() > 3) {
        lanes.emplace_back(lane);
      }
    }
  }
  return lanes;
}

bool ReferenceInfo::UpdateVehicleState(const vehicle_state::KinoDynamicState &vehicle_state) {
  std::lock_guard<std::mutex> lock_guard(vehicle_mutex_);
  vehicle_state_ = vehicle_state;
  has_vehicle_state_ = true;
  return true;
}

bool ReferenceInfo::UpdateReferenceLine(const std::vector<ReferenceLine> &reference_lines) {
  if (reference_lines.empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lock_guard(reference_line_mutex_);
  ref_lines_.assign(reference_lines.begin(), reference_lines.end());
  reference_line_history_.push_back(ref_lines_);
  return false;
}

bool ReferenceInfo::GetReferenceLines(std::vector<ReferenceLine> *reference_lines) {
  std::lock_guard<std::mutex> lock_guard(reference_line_mutex_);
  if (!ref_lines_.empty()) {
    reference_lines->assign(ref_lines_.begin(), ref_lines_.end());
    return true;
  } else {
    if (reference_line_history_.empty()) {
      return false;
    }
    reference_lines->assign(reference_line_history_.back().begin(), reference_line_history_.back().end());
    return true;
  }
  return false;
}

void ReferenceInfo::GenerateThread() {
  while (!is_stop_) {
    static constexpr int32_t kSleepTime = 50;  // milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTime));
    auto start_time = std::chrono::system_clock::now();
    if (!has_route_) {
      ROS_FATAL("Routing is not ready.");
      continue;
    }
    std::vector<ReferenceLine> ref_lines;
    if (!CreateReferenceLines(true, ref_lines)) {
      ROS_FATAL("Failed to create ReferenceLines");
      continue;
    }
    UpdateReferenceLine(ref_lines);
    auto end_time = std::chrono::system_clock::now();
    std::lock_guard<std::mutex> lock(reference_line_mutex_);
  }
}
void ReferenceInfo::Stop() {
  is_stop_ = true;
  task_future_.get();

}
}

