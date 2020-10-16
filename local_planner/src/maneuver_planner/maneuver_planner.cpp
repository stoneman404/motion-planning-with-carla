
#include <planning_srvs/Route.h>
#include <reference_line/reference_line.hpp>
#include <tf/transform_datatypes.h>
#include "maneuver_planner/maneuver_planner.hpp"
#include "maneuver_planner/state.hpp"
#include "maneuver_planner/follow_lane_state.hpp"
#include "maneuver_planner/change_right_lane_state.hpp"
#include "maneuver_planner/change_left_lane_state.hpp"
#include "maneuver_planner/stop_state.hpp"
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
  auto ego_pose = VehicleState::Instance().pose();
  auto destination = PlanningContext::Instance().global_goal_pose().pose;
  PlanningContext::Instance().mutable_route_infos().clear();
  PlanningContext::Instance().mutable_reference_lines().clear();
  ROS_ASSERT(this->ReRoute(ego_pose, destination, PlanningContext::Instance().mutable_route_infos().front()));
  ROS_ASSERT(this->GenerateReferenceLine(PlanningContext::Instance().route_infos().front(),
                                         PlanningContext::Instance().mutable_reference_lines().front()));
  current_lane_id_ = VehicleState::Instance().lane_id();
  current_state_.reset(&FollowLaneState::Instance());
  maneuver_goal_.decision_type = DecisionType::kFollowLane;
  maneuver_goal_.maneuver_infos.resize(1);
  maneuver_goal_.maneuver_infos.front().maneuver_target.target_speed = PlanningConfig::Instance().target_speed();
  maneuver_goal_.maneuver_infos.front().ptr_ref_line = PlanningContext::Instance().reference_lines().front();
  maneuver_goal_.maneuver_infos.front().has_stop_point = false;
  maneuver_goal_.maneuver_infos.front().lane_id = current_lane_id_;
  ROS_ASSERT(current_state_->Enter(this));
}

bool ManeuverPlanner::Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                              planning_msgs::Trajectory::Ptr pub_trajectory) {
  if (current_state_ == nullptr) {
    ROS_FATAL("[ManeuverPlanner::Process], the current state is nullptr");
    return false;
  }

  ROS_DEBUG("[ManeuverPlanner::Process], current state is [%s]", current_state_->Name().c_str());
  if (!current_state_->Execute(this)) {
    ROS_FATAL("[ManeuverPlanner::Process]");
    return false;
  }
  std::unique_ptr<State> state(current_state_->NextState(this));

  if (state != nullptr && state->Name() != current_state_->Name()) {
    current_state_->Exit(this);
    current_state_ = std::move(state);
    current_state_->Enter(this);
  }
  return true;
}

bool ManeuverPlanner::UpdateRouteInfo() {
  if (this->NeedReRoute()) {
    auto ego_pose = VehicleState::Instance().ego_waypoint();
    auto destination = PlanningContext::Instance().global_goal_pose();
    const double theta = MathUtil::NormalizeAngle(tf::getYaw(ego_pose.pose.orientation));
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const double offset_length = VehicleState::Instance().ego_waypoint().lane_width;
    planning_srvs::RouteResponse route_response;
    switch (maneuver_goal_.decision_type) {
      case DecisionType::kChangeLeft: {
        auto new_pose = ego_pose.pose;
        new_pose.position.x = ego_pose.pose.position.x - sin_theta * offset_length;
        new_pose.position.y = ego_pose.pose.position.y + cos_theta * offset_length;
        this->ReRoute(ego_pose.pose, destination.pose, route_response);
        break;
      }
      case DecisionType::kChangeRight: {
        auto new_pose = ego_pose.pose;
        new_pose.position.x = ego_pose.pose.position.x + sin_theta * offset_length;
        new_pose.position.y = ego_pose.pose.position.y + cos_theta * offset_length;
        this->ReRoute(ego_pose.pose, destination.pose, route_response);
        break;
      }
      case DecisionType::kStopAtDestination:
      case DecisionType::kStopAtTrafficSign:
      case DecisionType::kEmergencyStop:
      case DecisionType::kFollowLane: {
        this->ReRoute(ego_pose.pose, destination.pose, route_response);
        break;
      }
      default: break;
    }
    PlanningContext::Instance().mutable_route_infos().push_back(route_response);
  }
  auto &routes = PlanningContext::Instance().mutable_route_infos();
  for (const auto &route : routes) {

  }
  return false;
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
    ROS_DEBUG("[Planner::Reroute], Reroute SUCCESSFUL, the route size is %zu", response.route.size());
    return true;
  }
}

bool ManeuverPlanner::GenerateReferenceLine(const planning_srvs::RouteResponse &route,
                                            std::shared_ptr<ReferenceLine> reference_line) {
  if (reference_line == nullptr) {
    return false;
  }
  const auto vehicle_pose = VehicleState::Instance().pose();
  const double max_forward_distance = PlanningConfig::Instance().reference_max_forward_distance();
  const double max_backward_distance = PlanningConfig::Instance().reference_max_backward_distance();
  const int matched_index = GetNearestIndex(vehicle_pose, route.route);
  const int start_index = GetStartIndex(matched_index, max_backward_distance, route.route);
  const int end_index = GetEndIndex(matched_index, max_forward_distance, route.route);
  if (end_index - start_index < 1) {
    return false;
  }
  const auto sample_way_points = GetWayPointsFromStartToEndIndex(start_index, end_index, route.route);
  if (sample_way_points.empty()) {
    ROS_FATAL("[GenerateReferenceLine], failed to get sampled way points");
    return false;
  }
  reference_line.reset(new ReferenceLine(route.route));
  if (!reference_line->Smooth()) {
    ROS_DEBUG("[ManeuverPlanner::GenerateReferenceLine], failed to smooth reference line.");
  }
  return true;
}

int ManeuverPlanner::GetNearestIndex(const geometry_msgs::Pose &ego_pose,
                                     const std::vector<planning_msgs::WayPoint> &way_points) {

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

int ManeuverPlanner::GetStartIndex(const int matched_index, double backward_distance,
                                   const std::vector<planning_msgs::WayPoint> &way_points) {
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
                                 const std::vector<planning_msgs::WayPoint> &way_points) {
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
    const std::vector<planning_msgs::WayPoint> &way_points) {
  if (start_index >= end_index) {
    ROS_ERROR("[ManeuverPlanner::GetWayPointsFromStartToEndIndex], "
              "Failed to get waypoints because start_index >= end_index");
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

void ManeuverPlanner::SetManeuverGoal(const ManeuverGoal &maneuver_goal) {
  this->maneuver_goal_ = maneuver_goal;
}

bool ManeuverPlanner::NeedReRoute() const {
  if (PlanningContext::Instance().route_infos().empty()
      || PlanningContext::Instance().reference_lines().empty()) {
    return true;
  }
  if (current_state_->Name() == "FollowLaneState" &&
      (maneuver_goal_.decision_type == DecisionType::kChangeLeft
          || maneuver_goal_.decision_type == DecisionType::kChangeRight)) {
    return true;
  }
  return false;
}

const ManeuverGoal &ManeuverPlanner::maneuver_goal() const {
  return maneuver_goal_;
}

ManeuverGoal &ManeuverPlanner::multable_maneuver_goal() { return maneuver_goal_; }
}