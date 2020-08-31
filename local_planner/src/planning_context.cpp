#include <reference_line/reference_line.hpp>
#include "planning_context.hpp"

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

const LocalGoal &PlanningContext::local_goal() const {
  return local_goal_;
}

void PlanningContext::UpdateLocalGoal(int lane_id, bool has_stop_point,
                                      double stop_s, double cruise_speed) {
  local_goal_.lane_id = lane_id;
  local_goal_.cruise_speed = cruise_speed;
  local_goal_.has_stop_point = has_stop_point;
  local_goal_.stop_s = stop_s;
}

void PlanningContext::UpdateLocalGoal(const planning::LocalGoal &local_goal) {
  this->local_goal_ = local_goal;
}

void PlanningContext::UpdateGlobalGoalPose(const geometry_msgs::PoseStamped &goal_pose) {
  this->global_goal_pose_ = goal_pose;
}

void PlanningContext::UpdateGlobalInitPose(const geometry_msgs::PoseWithCovarianceStamped &init_pose) {
  this->global_init_pose_ = init_pose;
}

const geometry_msgs::PoseWithCovarianceStamped& PlanningContext::global_init_pose() const {
  return this->global_init_pose_;
}

const geometry_msgs::PoseStamped &PlanningContext::global_goal_pose() const {
  return global_goal_pose_;
}

}