#include <tf/transform_datatypes.h>
#include "behaviour.hpp"
namespace planning {

void ProbDistributeOfLatBehaviour::SetEntry(const LateralBehaviour &behaviour,
                                            double prob,
                                            const std::shared_ptr<ReferenceLine> &lane) {

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
                                                              std::shared_ptr<ReferenceLine> &lane) const {
  if (prob_dist_queue_.empty()) {
    return false;
  }
  behaviour = prob_dist_queue_.top().first.first;
  lane = prob_dist_queue_.top().first.second;
  return true;
}

bool ProbDistributeOfLatBehaviour::GetKthMaxProbBehavioursAndLanes(
    uint32_t k,
    std::vector<std::pair<LateralBehaviour, std::shared_ptr<ReferenceLine>>> &behaviour_lane_pairs) {
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
ReferenceInfo::ReferenceInfo(double max_curvature,
                             double deviation_weight,
                             double heading_weight,
                             double length_weight,
                             std::vector<planning_msgs::WayPoint> main_lane,
                             std::vector<std::vector<planning_msgs::WayPoint>> left_lanes,
                             std::vector<std::vector<planning_msgs::WayPoint>> right_lanes)
    : reference_smooth_max_curvature_(max_curvature),
      reference_smooth_deviation_weight_(deviation_weight),
      reference_smooth_heading_weight_(heading_weight),
      reference_smooth_length_weight_(length_weight),
      main_lane_(std::move(main_lane)),
      left_lanes_(std::move(left_lanes)),
      right_lanes_(std::move(right_lanes)) {}

bool ReferenceInfo::GetReferenceLines(const vehicle_state::KinoDynamicState &vehicle_state,
                                      double lookahead_distance,
                                      double lookback_distance,
                                      bool smooth,
                                      std::vector<std::shared_ptr<ReferenceLine>> &ref_lanes) {
  auto main_ref_lane = std::make_shared<ReferenceLine>();
  auto result = ReferenceInfo::RetriveReferenceLineFromRoute(
      main_ref_lane, vehicle_state,
      main_lane_,
      lookahead_distance,
      lookback_distance,
      smooth,
      reference_smooth_max_curvature_,
      reference_smooth_deviation_weight_,
      reference_smooth_heading_weight_,
      reference_smooth_length_weight_);
  if (!result) {
    return false;
  }
  ref_lanes.emplace_back(main_ref_lane);
  bool has_overlap_left_lane = false;
  size_t overlap_left_index = 0;
  for (size_t i = 0; i < left_lanes_.size(); ++i) {
    auto left_lane = left_lanes_[i];
    double overlap_begin_s, overlap_end_s;
    if (HasOverLapWithRefLane(main_ref_lane, left_lane, &overlap_begin_s, &overlap_end_s)) {
      overlap_left_index = i;
      has_overlap_left_lane = true;
      break;
    }
  }
  if (has_overlap_left_lane) {
    auto left_ref_lane = std::make_shared<ReferenceLine>();
    if (RetriveReferenceLineFromRoute(
        left_ref_lane, vehicle_state,
        left_lanes_[overlap_left_index],
        lookback_distance,
        lookback_distance,
        smooth,
        reference_smooth_max_curvature_,
        reference_smooth_deviation_weight_,
        reference_smooth_heading_weight_,
        reference_smooth_length_weight_)) {
      ref_lanes.emplace_back(left_ref_lane);
    }
  }

  bool has_overlap_right_lane = false;
  size_t overlap_right_index = 0;
  for (size_t i = 0; i < right_lanes_.size(); ++i) {
    auto right_lane = right_lanes_[i];
    double overlap_begin_s, overlap_end_s;
    if (HasOverLapWithRefLane(main_ref_lane, right_lane, &overlap_begin_s, &overlap_end_s)) {
      overlap_right_index = i;
      has_overlap_right_lane = true;
      break;
    }
  }
  if (has_overlap_right_lane) {
    auto right_ref_lane = std::make_shared<ReferenceLine>();
    if (RetriveReferenceLineFromRoute(
        right_ref_lane, vehicle_state,
        right_lanes_[overlap_right_index],
        lookback_distance,
        lookback_distance,
        smooth,
        reference_smooth_max_curvature_,
        reference_smooth_deviation_weight_,
        reference_smooth_heading_weight_,
        reference_smooth_length_weight_)) {
      ref_lanes.emplace_back(right_ref_lane);
    }
  }
  return true;
}

bool ReferenceInfo::RetriveReferenceLineFromRoute(std::shared_ptr<ReferenceLine> &ref_lane,
                                                  const vehicle_state::KinoDynamicState &vehicle_state,
                                                  const std::vector<planning_msgs::WayPoint> &lane,
                                                  double lookahead_distance,
                                                  double lookback_distance,
                                                  bool smooth,
                                                  double max_curvature,
                                                  double deviation_weight,
                                                  double heading_weight,
                                                  double length_weight) {
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
  auto main_ref_lane = std::make_shared<ReferenceLine>(sampled_way_points);
  if (smooth) {
    if (!main_ref_lane->Smooth(deviation_weight,
                               heading_weight,
                               length_weight,
                               max_curvature)) {
      ROS_WARN("Failed to Smooth Reference Line");
    }
  }

  return true;
}
bool ReferenceInfo::HasOverLapWithRefLane(const std::shared_ptr<ReferenceLine> &ref_lane,
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
  if (!ref_lane->XYToSL(begin.pose.position.x, begin.pose.position.y, &begin_sl)) {
    return false;
  }
  if (begin_sl.s > ref_lane->Length() - std::numeric_limits<double>::epsilon()) {
    return false;
  }
  auto begin_ref_point = ref_lane->GetReferencePoint(begin_sl.s);
  const double begin_angle_diff =
      common::MathUtils::CalcAngleDist(common::MathUtils::NormalizeAngle(tf::getYaw(begin.pose.orientation)),
                                       begin_ref_point.theta());
  if (std::fabs(begin_angle_diff) > kMaxAngleDiff) {
    return false;
  }
  if (!ref_lane->XYToSL(end.pose.position.x, end.pose.position.y, &end_sl)) {
    return false;
  }
  if (end_sl.s < std::numeric_limits<double>::epsilon()) {
    return false;
  }
  auto end_ref_point = ref_lane->GetReferencePoint(begin_sl.s);
  const double end_angle_diff =
      common::MathUtils::CalcAngleDist(common::MathUtils::NormalizeAngle(tf::getYaw(end.pose.orientation)),
                                       end_ref_point.theta());
  if (std::fabs(end_angle_diff) > kMaxAngleDiff) {
    return false;
  }
  *overlap_start_s = begin_sl.s < std::numeric_limits<double>::epsilon() ? 0.0 : begin_sl.s;
  *overlap_end_s =
      end_sl.s > ref_lane->Length() - std::numeric_limits<double>::epsilon() ? ref_lane->Length() : end_sl.s;
  return true;
}
}