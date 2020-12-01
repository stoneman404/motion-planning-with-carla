#include <planning_config.hpp>
#include <tf/transform_datatypes.h>
#include <agent/behaviour.hpp>
#include <agent/agent.hpp>
#include "behaviour_planner/behaviour_strategy.hpp"
#include "mpdm_planner.hpp"

namespace planning {
MPDMPlanner::MPDMPlanner(const ros::NodeHandle &nh, common::ThreadPool *thread_pool)
    : nh_(nh), thread_pool_(thread_pool) {
  behavior_.longitudinal_behaviour = LongitudinalBehaviour::kStopping;
  behavior_.lateral_behaviour = LateralBehaviour::kLaneKeeping;
  behavior_.desired_velocity_ = 0.0;
  behavior_.forward_behaviours = decltype(behavior_.forward_behaviours)();
}

bool MPDMPlanner::Execute(const planning_msgs::TrajectoryPoint &init_point, Behaviour &behaviour) {
  if (!this->UpdateAvailablePoliciesAndRefLanes()) {
    ROS_FATAL("[MPDMPlanner::Execute], failed to [UpdateAvailablePoliciesAndRefLanes]");
    return false;
  }
  if (available_policies_with_ref_lanes_.empty()) {
    ROS_ERROR("[MPDMPlanner::Execute], no available policy found");
    return false;
  }

  return true;
}

void MPDMPlanner::UpdateAgentsMostLikelyPolicies(const std::vector<Agent> &agents) {
  auto agents_vec = agents;
  behaviour_agent_set_ = decltype(behaviour_agent_set_)();
  for (auto &agent : agents_vec) {
    if (!GetAgentMostLikelyPolicyAndReferenceLane(agent)) {
      continue;
    }
    behaviour_agent_set_.insert(std::make_pair(agent.id(), agent));
    if (agent.is_host()) {
      ego_agent_ = agent;
    }
  }
}

bool MPDMPlanner::GetReferenceLane(const geometry_msgs::Pose &current_pose,
                                   const LateralBehaviour &lateral_behaviour,
                                   double forward_length,
                                   double backward_length,
                                   bool need_to_smooth,
                                   std::shared_ptr<ReferenceLine> &ref_line) {
  geometry_msgs::Pose end_pose;
  planning_srvs::RoutePlanServiceResponse response;
  if (!GetRoute(current_pose, end_pose, lateral_behaviour, response)) {
    ROS_FATAL("[Failed to GetRoute]");
    return false;
  }
//  double min_index = 0;
//  double min_dist = std::numeric_limits<double>::max();
//
//  // find nearest waypoint in route
//  for (size_t i = 0; i < response.route.size(); ++i) {
//    auto waypoint = response.route.at(i);
//    double distance = std::pow(waypoint.pose.position.x - current_pose.position.x, 2) +
//        std::pow(waypoint.pose.position.y - current_pose.position.y, 2);
//    if (distance < min_dist) {
//      min_dist = distance;
//      min_index = i;
//    }
//  }
//
//  double s = 0.0;
//  // get start index
//  size_t current_index = min_index;
//  while (s < backward_length && current_index - 1 >= 0) {
//    auto cur_waypoint = response.route.at(current_index);
//    auto prev_waypoint = response.route.at(current_index - 1);
//    s += std::hypot(cur_waypoint.pose.position.x - prev_waypoint.pose.position.x,
//                    cur_waypoint.pose.position.y - prev_waypoint.pose.position.y);
//    current_index = current_index - 1;
//  }
//  size_t start_index = current_index;
//  // get end index
//  current_index = min_index;
//  s = 0.0;
//  while (s < forward_length && current_index + 1 < response.route.size()) {
//    auto cur_waypoint = response.route.at(current_index);
//    auto next_waypoint = response.route.at(current_index + 1);
//    s += std::hypot(cur_waypoint.pose.position.x - next_waypoint.pose.position.x,
//                    cur_waypoint.pose.position.y - next_waypoint.pose.position.y);
//    current_index = current_index + 1;
//  }
//  size_t end_index = current_index;
//  if (end_index - start_index < 3) {
//    ROS_FATAL("the reference line is too short, end_index - start_index =%ld", end_index - start_index);
//    return false;
//  }
//  std::vector<planning_msgs::WayPoint> waypoints;
//  waypoints.reserve(end_index - start_index + 1);
//  for (size_t i = start_index; i <= end_index; ++i) {
//    waypoints.push_back(response.route.at(i));
//  }
  ref_line = std::make_shared<ReferenceLine>(response.route);
  if (need_to_smooth) {
    auto result = ref_line->Smooth(PlanningConfig::Instance().reference_smoother_deviation_weight(),
                                   PlanningConfig::Instance().reference_smoother_heading_weight(),
                                   PlanningConfig::Instance().reference_smoother_distance_weight(),
                                   PlanningConfig::Instance().reference_smoother_max_curvature());
    if (!result) {
      ROS_WARN("the smooth reference line is failed");
    }
  }
  return true;
}

bool MPDMPlanner::GetRoute(const geometry_msgs::Pose &start,
                           const geometry_msgs::Pose &destination,
                           const LateralBehaviour &lateral_behaviour,
                           planning_srvs::RoutePlanServiceResponse &route_response) {

  planning_srvs::RoutePlanService srv;
  switch (lateral_behaviour) {
    case LateralBehaviour::kLaneChangeLeft:srv.request.offset = 1;
      break;
    case LateralBehaviour::kLaneChangeRight:srv.request.offset = -1;
      break;
    case LateralBehaviour::kUndefined:
    case LateralBehaviour::kLaneKeeping:
    default:srv.request.offset = 0;
  }
//    srv.request.offset = 0;
  srv.request.start_pose = start;
  srv.request.end_pose = destination;
  if (!route_service_client_.call(srv)) {
    return false;
  }
  route_response = std::move(srv.response);
  return true;
}

bool MPDMPlanner::GetAgentMostLikelyPolicyAndReferenceLane(Agent &agent) {
  // 1. first get current ref lane for agent
  geometry_msgs::Pose current_pose;
  const auto state = agent.state();
  current_pose = agent.way_point().pose;
  std::shared_ptr<ReferenceLine> ref_lane = std::make_shared<ReferenceLine>();
  if (!this->GetReferenceLane(current_pose, LateralBehaviour::kLaneKeeping,
                              PlanningConfig::Instance().reference_max_forward_distance(),
                              PlanningConfig::Instance().reference_max_backward_distance(),
                              false, ref_lane)) {
    agent.set_most_likely_behaviour(LateralBehaviour::kUndefined);
    return false;

  }
  agent.set_current_ref_lane(ref_lane);
  // get the most likely policies for agent
  agent.PredictAgentBehaviour();
  switch (agent.most_likely_behaviour()) {
    case LateralBehaviour::kLaneKeeping: {
      agent.set_target_ref_lane(ref_lane);
      break;
    }
    case LateralBehaviour::kLaneChangeRight: {
      std::shared_ptr<ReferenceLine> right_ref_lane = std::make_shared<ReferenceLine>();
      if (!this->GetReferenceLane(current_pose, LateralBehaviour::kLaneChangeRight,
                                  PlanningConfig::Instance().reference_max_forward_distance(),
                                  PlanningConfig::Instance().reference_max_backward_distance(),
                                  false, right_ref_lane)) {
        agent.set_most_likely_behaviour(LateralBehaviour::kLaneKeeping);
        agent.set_target_ref_lane(agent.current_ref_lane());
      }
      agent.set_target_ref_lane(right_ref_lane);
      break;
    }
    case LateralBehaviour::kLaneChangeLeft: {
      std::shared_ptr<ReferenceLine> left_lane = std::make_shared<ReferenceLine>();
      if (!this->GetReferenceLane(current_pose, LateralBehaviour::kLaneChangeLeft,
                                  PlanningConfig::Instance().reference_max_forward_distance(),
                                  PlanningConfig::Instance().reference_max_backward_distance(),
                                  false, left_lane)) {
        agent.set_most_likely_behaviour(LateralBehaviour::kLaneKeeping);
        agent.set_target_ref_lane(agent.current_ref_lane());
      }
      agent.set_target_ref_lane(left_lane);
      break;
    }
    case LateralBehaviour::kUndefined:
    default: {
      agent.set_target_ref_lane(ref_lane);
      break;
    }
  }
  return true;
}

bool MPDMPlanner::UpdateAvailablePoliciesAndRefLanes() {
  available_policies_with_ref_lanes_.insert({LateralBehaviour::kLaneKeeping, ego_agent_.current_ref_lane()});
  switch (ego_agent_.most_likely_behaviour()) {
    case LateralBehaviour::kLaneChangeLeft: {
      available_policies_with_ref_lanes_.insert({LateralBehaviour::kLaneChangeLeft, ego_agent_.target_ref_lane()});
      common::SLPoint sl_point;

      if (!ego_agent_.current_ref_lane()->XYToSL(ego_agent_.state().x_, ego_agent_.state().y_, &sl_point)) {
        break;
      }
      if (ego_agent_.current_ref_lane()->CanChangeRight(sl_point.s)) {
        geometry_msgs::Pose cur_pose;
        std::shared_ptr<ReferenceLine> ref_lane;
        cur_pose.position.x = ego_agent_.state().x_;
        cur_pose.position.y = ego_agent_.state().y_;
        cur_pose.orientation = tf::createQuaternionMsgFromYaw(ego_agent_.state().theta_);
        if (GetReferenceLane(cur_pose,
                             LateralBehaviour::kLaneChangeRight,
                             PlanningConfig::Instance().reference_max_forward_distance(),
                             PlanningConfig::Instance().reference_max_backward_distance(),
                             false,
                             ref_lane)) {
          available_policies_with_ref_lanes_.insert({LateralBehaviour::kLaneChangeRight, ref_lane});
        }
      }
      break;
    }
    case LateralBehaviour::kLaneChangeRight: {
      available_policies_with_ref_lanes_.insert({LateralBehaviour::kLaneChangeRight, ego_agent_.target_ref_lane()});
      common::SLPoint sl_point;
      if (!ego_agent_.current_ref_lane()->XYToSL(ego_agent_.state().x_, ego_agent_.state().y_, &sl_point)) {
        break;
      }
      if (ego_agent_.current_ref_lane()->CanChangeLeft(sl_point.s)) {
        geometry_msgs::Pose cur_pose;
        std::shared_ptr<ReferenceLine> ref_lane;
        cur_pose.position.x = ego_agent_.state().x_;
        cur_pose.position.y = ego_agent_.state().y_;
        cur_pose.orientation = tf::createQuaternionMsgFromYaw(ego_agent_.state().theta_);
        if (GetReferenceLane(cur_pose,
                             LateralBehaviour::kLaneChangeLeft,
                             PlanningConfig::Instance().reference_max_forward_distance(),
                             PlanningConfig::Instance().reference_max_backward_distance(),
                             false,
                             ref_lane)) {
          available_policies_with_ref_lanes_.insert({LateralBehaviour::kLaneChangeLeft, ref_lane});
        }
      }
      break;
    }
    case LateralBehaviour::kLaneKeeping:
    case LateralBehaviour::kUndefined:
    default:break;
  }
  return true;

}
bool MPDMPlanner::MultiPoliciesDecision(Behaviour &mpdm_decision, double &desired_velocity) {
//   size_t available_policies_num = available_policies_with_ref_lanes_.size();

  for (const auto &policy : available_policies_with_ref_lanes_) {
    planning_msgs::Trajectory trajectory;
    std::pair<int, planning_msgs::Trajectory> surround_trajs;
    if (!SimulateEgoAgentPolicy(policy.first, trajectory, surround_trajs)) {

      continue;
    }
  }

}
bool MPDMPlanner::SimulateEgoAgentPolicy(const LateralBehaviour &policy,
                                         planning_msgs::Trajectory &ego_traj,
                                         std::pair<int, planning_msgs::Trajectory> &surrounding_trajs) {

  return false;
}
bool MPDMPlanner::CloseLoopSimForward(const LateralBehaviour &policy,
                                      planning_msgs::Trajectory &ego_traj,
                                      std::pair<int, planning_msgs::Trajectory> &surrounding_trajs) {

  return false;
}

}