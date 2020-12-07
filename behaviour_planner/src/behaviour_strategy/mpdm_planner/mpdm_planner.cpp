#include <tf/transform_datatypes.h>
#include <agent/behaviour.hpp>
#include <agent/agent.hpp>
#include "mpdm_planner.hpp"
#include "name/string_name.hpp"
namespace planning {
MPDMPlanner::MPDMPlanner(const ros::NodeHandle &nh,
                         const PolicySimulateConfig &config,
                         common::ThreadPool *thread_pool)
    : nh_(nh), thread_pool_(thread_pool) {
  behavior_.longitudinal_behaviour = LongitudinalBehaviour::STOPPING;
  behavior_.lateral_behaviour = LateralBehaviour::LANE_KEEPING;
  behavior_.forward_behaviours = decltype(behavior_.forward_behaviours)();
  behavior_.forward_trajs = decltype(behavior_.forward_trajs)();
  route_service_client_ = nh_.serviceClient<planning_srvs::RoutePlanService>(common::service::kRouteServiceName);
  policy_decider_ = std::make_unique<PolicyDecider>(config);
}

bool MPDMPlanner::Execute(Behaviour &behaviour) {
  if (!this->UpdateAvailablePoliciesAndRefLanes()) {
    ROS_FATAL("[MPDMPlanner::Execute], failed to [UpdateAvailablePoliciesAndRefLanes]");
    return false;
  }
  if (available_policies_with_ref_lanes_.empty()) {
    ROS_ERROR("[MPDMPlanner::Execute], no available policy found");
    return false;
  }
  std::vector<std::pair<LateralBehaviour, std::shared_ptr<ReferenceLine>>> possible_policies;
  std::pair<LateralBehaviour, std::shared_ptr<ReferenceLine>> best_policy;
  std::vector<std::pair<LateralBehaviour, std::shared_ptr<ReferenceLine>>> forward_policies;
  std::vector<std::unordered_map<int, planning_msgs::Trajectory>> surround_trajs;
  std::vector<planning_msgs::Trajectory> forward_trajectories;
  for (const auto &policy : available_policies_with_ref_lanes_) {
    possible_policies.emplace_back(policy);
  }
  if (!policy_decider_->PolicyDecision(ego_agent_, behaviour_agent_set_, possible_policies, best_policy,
                                       forward_policies, surround_trajs, forward_trajectories)) {
    return false;
  }
  behaviour.lateral_behaviour = best_policy.first;
  behaviour.forward_behaviours = forward_policies;
  behaviour.forward_trajs = forward_trajectories;
  behaviour.surrounding_trajs = surround_trajs;
  return true;
}

void MPDMPlanner::SetAgentSet(const std::unordered_map<int, Agent> &agent_set) {
//  auto agents_vec = agent_set;
  behaviour_agent_set_ = agent_set;
  for (auto &agent : behaviour_agent_set_) {
    if (!GetAgentMostLikelyPolicyAndReferenceLane(agent.second)) {
      continue;
    }
    if (agent.second.is_host()) {
      ego_agent_ = agent.second;
    }
  }
}

bool MPDMPlanner::GetReferenceLane(const geometry_msgs::Pose &current_pose,
                                   const LateralBehaviour &lateral_behaviour,
                                   bool need_to_smooth,
                                   std::shared_ptr<ReferenceLine> &ref_line) {
  geometry_msgs::Pose end_pose;
  planning_srvs::RoutePlanServiceResponse response;
  if (!GetRoute(current_pose, end_pose, lateral_behaviour, response)) {
    ROS_FATAL("[Failed to GetRoute]");
    return false;
  }
  ref_line = std::make_shared<ReferenceLine>(response.route);
  return true;
}

bool MPDMPlanner::GetRoute(const geometry_msgs::Pose &start,
                           const geometry_msgs::Pose &destination,
                           const LateralBehaviour &lateral_behaviour,
                           planning_srvs::RoutePlanServiceResponse &route_response) {

  planning_srvs::RoutePlanService srv;
  switch (lateral_behaviour) {
    case LateralBehaviour::LANE_CHANGE_LEFT: {
      srv.request.offset = 1;
      break;
    }
    case LateralBehaviour::LANE_CHANGE_RIGHT: {
      srv.request.offset = -1;
      break;
    }
    case LateralBehaviour::UNDEFINED:
    case LateralBehaviour::LANE_KEEPING:
    default: {
      srv.request.offset = 0;
      break;
    }
  }
//    srv.request.offset = 0;
  srv.request.start_pose = start;
  srv.request.end_pose = destination;
  if (!route_service_client_.call(srv)) {
    return false;
  }
  route_response = srv.response;
  return true;
}

bool MPDMPlanner::GetAgentMostLikelyPolicyAndReferenceLane(Agent &agent) {
  // 1. first get current ref lane for agent
  if (agent.agent_type() == AgentType::VEHICLE) {
    geometry_msgs::Pose current_pose;
    current_pose.position.x = agent.state().x_;
    current_pose.position.y = agent.state().y_;
    current_pose.position.z = agent.state().z_;
    current_pose.orientation = tf::createQuaternionMsgFromYaw(agent.state().theta_);
    std::shared_ptr<ReferenceLine> ref_lane = std::make_shared<ReferenceLine>();
    if (!this->GetReferenceLane(current_pose, LateralBehaviour::LANE_KEEPING,
                                false, ref_lane)) {
      agent.set_most_likely_behaviour(LateralBehaviour::UNDEFINED);
      return false;

    }
    agent.set_current_ref_lane(ref_lane);
    // get the most likely policies for agent
    agent.PredictAgentBehaviour();
    switch (agent.most_likely_behaviour()) {
      case LateralBehaviour::LANE_KEEPING: {
        agent.set_target_ref_lane(ref_lane);
        break;
      }
      case LateralBehaviour::LANE_CHANGE_RIGHT: {
        std::shared_ptr<ReferenceLine> right_ref_lane = std::make_shared<ReferenceLine>();
        if (!this->GetReferenceLane(current_pose, LateralBehaviour::LANE_CHANGE_RIGHT,
                                    false, right_ref_lane)) {
          agent.set_most_likely_behaviour(LateralBehaviour::LANE_KEEPING);
          agent.set_target_ref_lane(agent.current_ref_lane());
        }
        agent.set_target_ref_lane(right_ref_lane);
        break;
      }
      case LateralBehaviour::LANE_CHANGE_LEFT: {
        std::shared_ptr<ReferenceLine> left_lane = std::make_shared<ReferenceLine>();
        if (!this->GetReferenceLane(current_pose, LateralBehaviour::LANE_CHANGE_LEFT,
                                    false, left_lane)) {
          agent.set_most_likely_behaviour(LateralBehaviour::LANE_KEEPING);
          agent.set_target_ref_lane(agent.current_ref_lane());
        }
        agent.set_target_ref_lane(left_lane);
        break;
      }
      case LateralBehaviour::UNDEFINED:
      default: {
        agent.set_most_likely_behaviour(LateralBehaviour::LANE_KEEPING);
        agent.set_target_ref_lane(ref_lane);
        break;
      }
    }
  }
  return true;
}

bool MPDMPlanner::UpdateAvailablePoliciesAndRefLanes() {
  available_policies_with_ref_lanes_.insert({LateralBehaviour::LANE_KEEPING, ego_agent_.current_ref_lane()});
  switch (ego_agent_.most_likely_behaviour()) {
    case LateralBehaviour::LANE_CHANGE_LEFT: {
      available_policies_with_ref_lanes_.insert({LateralBehaviour::LANE_CHANGE_LEFT, ego_agent_.target_ref_lane()});
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
                             LateralBehaviour::LANE_CHANGE_RIGHT,
                             false,
                             ref_lane)) {
          available_policies_with_ref_lanes_.insert({LateralBehaviour::LANE_CHANGE_RIGHT, ref_lane});
        }
      }
      break;
    }
    case LateralBehaviour::LANE_CHANGE_RIGHT: {
      available_policies_with_ref_lanes_.insert({LateralBehaviour::LANE_CHANGE_RIGHT, ego_agent_.target_ref_lane()});
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
                             LateralBehaviour::LANE_CHANGE_LEFT,
                             false,
                             ref_lane)) {
          available_policies_with_ref_lanes_.insert({LateralBehaviour::LANE_CHANGE_LEFT, ref_lane});
        }
      }
      break;
    }
    case LateralBehaviour::LANE_KEEPING:
    case LateralBehaviour::UNDEFINED:
    default: {
      break;
    }
  }
  return true;
}

}