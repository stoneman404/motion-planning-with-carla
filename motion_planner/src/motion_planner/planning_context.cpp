#include <reference_line/reference_line.hpp>
#include <carla_waypoint_types/CarlaWaypoint.h>
#include "motion_planner/planning_context.hpp"

namespace planning {

PlanningContext &PlanningContext::Instance() {
  static PlanningContext instance;
  return instance;
}

const std::list<planning_srvs::RouteResponse> &PlanningContext::route_infos() const {
  return route_infos_;
}

const std::list<std::shared_ptr<ReferenceLine>> &PlanningContext::reference_lines() const {
  return reference_lines_;
}

void PlanningContext::UpdateGlobalGoalPose(const geometry_msgs::PoseStamped &goal_pose) {
  this->global_goal_pose_ = goal_pose;
}

void PlanningContext::UpdateGlobalInitPose(const geometry_msgs::PoseWithCovarianceStamped &init_pose) {
  this->global_init_pose_ = init_pose;
}

const geometry_msgs::PoseWithCovarianceStamped &PlanningContext::global_init_pose() const {
  return this->global_init_pose_;
}

const geometry_msgs::PoseStamped &PlanningContext::global_goal_pose() const {
  return global_goal_pose_;
}

std::list<std::shared_ptr<ReferenceLine>> &PlanningContext::mutable_reference_lines() { return reference_lines_; }

std::list<planning_srvs::RouteResponse> &PlanningContext::mutable_route_infos() { return route_infos_; }
}