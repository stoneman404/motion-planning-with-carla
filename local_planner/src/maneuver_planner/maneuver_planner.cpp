#include <planning_srvs/Route.h>
#include <reference_line/reference_line.hpp>
#include "maneuver_planner/maneuver_planner.hpp"
#include "maneuver_planner/state.hpp"
#include "maneuver_planner/keep_lane_state.hpp"
#include "maneuver_planner/change_lane_left_state.hpp"
#include "maneuver_planner/change_lane_right_state.hpp"
#include "maneuver_planner/stop_at_sign.hpp"
#include "maneuver_planner/emergency_stop_state.hpp"
#include "string_name.hpp"
#include "planning_context.hpp"
#include "planning_config.hpp"
namespace planning {

ManeuverPlanner::ManeuverPlanner(const ros::NodeHandle &nh) : nh_(nh) {
  this->InitPlanner();
}

void ManeuverPlanner::InitPlanner() {
  this->route_service_client_ = nh_.serviceClient<planning_srvs::Route>(
      service::kRouteServiceName);
  current_lane_id_ = VehicleState::Instance().lane_id();
  current_state_.reset(&KeepLaneState::Instance());
  ROS_ASSERT(current_state_->Enter(this));
}

bool ManeuverPlanner::Process(const planning_msgs::TrajectoryPoint &init_trajectory_point) {
  if (current_state_ == nullptr) {
    ROS_FATAL("[ManeuverPlanner::Process], the current state is nullptr");
    return false;
  } else {
    if (!current_state_->Execute(this)) {
      ROS_FATAL("[ManeuverPlanner::Process]");
    }
    std::unique_ptr<State> state(current_state_->NextState(this));
    if (state != nullptr) {
      current_state_->Exit(this);
      current_state_ = std::move(state);
      current_state_->Enter(this);
    }
  }
  return true;
}

bool ManeuverPlanner::ReRoute(const geometry_msgs::Pose &start,
                              const geometry_msgs::Pose &destination,
                              planning_srvs::RouteResponse &response) {
  planning_srvs::Route srv;
  srv.request.start_pose = start;
  srv.request.end_pose = destination;
  if (!route_service_client_.call(srv)) {
    ROS_FATAL("[Planner::Reroute], Failed to ReRoute!");
    return false;
  } else {
    response = srv.response;
    ROS_INFO("[Planner::Reroute], Reroute SUCCESSFUL,"
             " the route size is %zu", response.route.size());
    return true;
  }
}

bool ManeuverPlanner::UpdateReferenceLine(std::list<std::shared_ptr<ReferenceLine>> *const reference_lines_list) const {
  const auto vehicle_pose = VehicleState::Instance().pose();
  const auto lane_id = VehicleState::Instance().lane_id();
  const double max_forward_distance = PlanningConfig::Instance().reference_max_forward_distance();
  const double max_backward_distance = PlanningConfig::Instance().reference_max_backward_distance();
  const auto route_infos = PlanningContext::Instance().route_infos();
  reference_lines_list->clear();
  for (const auto &route : route_infos) {
    const int matched_index = GetNearestIndex(vehicle_pose, route.route);
    const int start_index = GetStartIndex(matched_index, max_backward_distance, route.route);
    const int end_index = GetStartIndex(matched_index, max_forward_distance, route.route);
    if (end_index - start_index < 1) {
      return false;
    }
    const auto sample_way_points = GetWayPointsFromStartToEndIndex(start_index, end_index, route.route);
    if (sample_way_points.empty()) {
      return false;
    }
    auto reference_line = std::make_shared<ReferenceLine>(sample_way_points);
    if (!reference_line->Smooth()) {
      ROS_WARN("[ManeuverPlanner::UpdateReferenceLine], Failed to smooth reference line");
    }
    reference_lines_list->push_back(reference_line);
  }
  return true;
}

int ManeuverPlanner::GetNearestIndex(const geometry_msgs::Pose &ego_pose,
                                     const std::vector<planning_msgs::WayPoint> &way_points) const {
  double min_distance = std::numeric_limits<double>::max();
  const double ego_x = ego_pose.position.x;
  const double ego_y = ego_pose.position.y;
  int min_index = 0;
  for (size_t i = 0; i < way_points.size(); ++i) {
    double x = way_points[i].pose.position.x;
    double y = way_points[i].pose.position.y;
    double dist = std::hypot(x - ego_x, y - ego_y);
    if (dist < min_distance) {
      min_distance = dist;
      min_index = i;
    }
  }
  return min_index;
}

int ManeuverPlanner::GetStartIndex(const int matched_index,
                                   double backward_distance,
                                   const std::vector<planning_msgs::WayPoint> &way_points) const {
  int current_index = matched_index;
  const auto matched_way_point = way_points[current_index];
  double s = 0;
  int last_index = current_index;

  while (current_index > 0 && s < backward_distance) {
    last_index = current_index - 1;
    const auto last_way_point = way_points[last_index];
    double ds = std::hypot(last_way_point.pose.position.x - matched_way_point.pose.position.x,
                           last_way_point.pose.position.y - matched_way_point.pose.position.y);
    s += ds;
    current_index--;
  }

  return last_index;

}

int ManeuverPlanner::GetEndIndex(const int matched_index,
                                 double forward_distance,
                                 const std::vector<planning_msgs::WayPoint> &way_points) const {
  int current_index = matched_index;
  const auto matched_way_point = way_points[current_index];
  double s = 0;
  int next_index = current_index;
  while (current_index < way_points.size() - 2 && s < forward_distance) {
    next_index = current_index + 1;
    const auto last_way_point = way_points[next_index];
    double ds = std::hypot(last_way_point.pose.position.x - matched_way_point.pose.position.x,
                           last_way_point.pose.position.y - matched_way_point.pose.position.y);
    s += ds;
    current_index++;
  }

  return next_index;

}
std::vector<planning_msgs::WayPoint> ManeuverPlanner::GetWayPointsFromStartToEndIndex(
    const int start_index,
    const int end_index,
    const std::vector<planning_msgs::WayPoint> &way_points) const {
  if (start_index >= end_index) {
    return {};
  }
  std::vector<planning_msgs::WayPoint> sampled_way_points;
  for (size_t i = start_index; i <= end_index; ++i) {
    sampled_way_points.push_back(way_points[i]);
  }
  return sampled_way_points;

}

const planning_msgs::TrajectoryPoint &ManeuverPlanner::init_trajectory_point() const {
  return init_trajectory_point_;
}

ManeuverPlanner::~ManeuverPlanner() {
  current_state_->Exit(this);
  current_state_.reset(nullptr);

}

}