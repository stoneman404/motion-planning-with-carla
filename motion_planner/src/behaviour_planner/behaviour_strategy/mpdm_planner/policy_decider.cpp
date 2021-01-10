#include "policy_decider.hpp"

namespace planning {

PolicyDecider::PolicyDecider(const PolicySimulateConfig &config)
    : config_(config),
      forward_simulator_(std::make_unique<OnLaneForwardSimulator>()) {
}

bool PolicyDecider::PolicyDecision(const Agent &ego_agent,
                                   const std::unordered_map<int, Agent> &agents_set,
                                   const PolicyDecider::Policies &possible_policies,
                                   Policy &best_policy,
                                   Policies &forward_policies,
                                   std::vector<SurroundingTrajectories> &forward_surrounding_trajectories,
                                   std::vector<planning_msgs::Trajectory> &forward_trajectories) {
  ego_id_ = ego_agent.id();
  agent_set_ = agents_set;

  std::vector<planning_msgs::Trajectory> valid_trajectories;
  std::vector<Policy> valid_policies;
  std::vector<SurroundingTrajectories> valid_surrounding_trajectories;
  std::unordered_map<int, SimulateAgent> simulate_agent_set;
  if (!PolicyDecider::GetSimulateAgentSet(agents_set, possible_policies, simulate_agent_set)) {
    ROS_FATAL("[PolicyDecider::GetSimulateAgentSet], Failed");
    return false;
  }

  if (simulate_agent_set.find(ego_id_) == simulate_agent_set.end()) {
    return false;
  }

  for (const auto &policy : possible_policies) {
    planning_msgs::Trajectory trajectory;
    SurroundingTrajectories surround_trajs;
    simulate_agent_set[ego_id_].behaviour_lane_pair = policy;

    if (!SimulateEgoAgentPolicy(simulate_agent_set, policy, trajectory, surround_trajs)) {
      ROS_WARN("[PolicyDecider::SimulateEgoAgentPolicy], "
               "failed to Simulate Policy : %i", static_cast<int>(policy.first));
      continue;
    }

    valid_trajectories.push_back(trajectory);
    valid_policies.push_back(policy);
    valid_surrounding_trajectories.push_back(surround_trajs);
  }
  if (valid_policies.size() != valid_trajectories.size()) {
    return false;
  }
  if (valid_trajectories.empty()) {
    return false;
  }
  Policy winner_policy;
  planning_msgs::Trajectory winner_forward_trajectory;
  double winner_score = 0.0;

  if (!EvaluateMultiPolicyTrajectories(valid_policies,
                                       valid_trajectories,
                                       valid_surrounding_trajectories,
                                       winner_policy,
                                       winner_forward_trajectory,
                                       winner_score)) {
    ROS_FATAL("[PolicyDecider::EvaluateMultiPolicyTrajectories], failed!!!");
    return false;
  }
  best_policy = winner_policy;
  forward_policies = valid_policies;
  forward_trajectories = valid_trajectories;
  forward_surrounding_trajectories = valid_surrounding_trajectories;
  return true;
}
bool PolicyDecider::GetSimulateAgentSet(const std::unordered_map<int, Agent> &agent_set,
                                        const PolicyDecider::Policies &possible_policies,
                                        std::unordered_map<int, SimulateAgent> &simulate_agent_set) {
  auto tmp_agent_set = agent_set;
  simulate_agent_set.clear();
  for (auto &agent : tmp_agent_set) {
    SimulateAgent simulate_agent;
    simulate_agent.agent = agent.second;
    if (!agent.second.is_host()) {
      std::vector<Policy> behaviour_with_lanes;
      if (!agent.second.GetKMaxProbBehavioursAndLanes(1, behaviour_with_lanes) || behaviour_with_lanes.empty()) {
        simulate_agent.behaviour_lane_pair.first = LateralBehaviour::UNDEFINED;
        simulate_agent.behaviour_lane_pair.second = ReferenceLine();
      } else {
        simulate_agent.behaviour_lane_pair = behaviour_with_lanes.front();
      }
    }
    simulate_agent_set.emplace(agent.first, simulate_agent);
  }
  return true;
}

bool PolicyDecider::SimulateEgoAgentPolicy(const std::unordered_map<int, SimulateAgent> &simulate_agent_set,
                                           const Policy &policy,
                                           planning_msgs::Trajectory &ego_traj,
                                           SurroundingTrajectories &surrounding_trajs) {

  ego_traj.trajectory_points.clear();
  surrounding_trajs.clear();
  auto timestamp = ros::Time::now();
  planning_msgs::TrajectoryPoint init_trajectory_point;
  for (const auto &simulate_agent : simulate_agent_set) {
    Agent::StateToTrajectoryPoint(simulate_agent.second.agent.state(), 0.0, init_trajectory_point);
    if (simulate_agent.second.agent.is_host()) {
      ego_traj.trajectory_points.push_back(init_trajectory_point);
      ego_traj.status = planning_msgs::Trajectory::NORMAL;
      ego_traj.header.stamp = timestamp;
      ego_traj.header.frame_id = "/map";
    } else {
      planning_msgs::Trajectory traj;
      traj.header.frame_id = "/map";
      traj.header.stamp = timestamp;
      traj.status = planning_msgs::Trajectory::NORMAL;
      traj.trajectory_points.push_back(init_trajectory_point);
      surrounding_trajs.emplace(simulate_agent.first, traj);
    }
  }
  if (!CloseLoopSimulate(policy, simulate_agent_set, ego_traj, surrounding_trajs)) {
    return false;
  }
  return true;
}
//
//bool PolicyDecider::OpenLoopSimForward(const Policy &policy,
//                                       const std::unordered_map<int, SimulateAgent> &simulate_agents,
//                                       planning_msgs::Trajectory &ego_traj,
//                                       std::unordered_map<int, planning_msgs::Trajectory> &surrounding_trajs) {
//
//  const int kForwardSteps = static_cast<int>(config_.sim_horizon_ / config_.sim_step_);
//  auto tmp_agent_set = simulate_agents;
//  for (int i = 0; i < kForwardSteps; ++i) {
//
//    for (auto &agent : tmp_agent_set) {
//      double desired_velocity = this->GetDesiredSpeed({agent.second.agent.state().x, agent.second.agent.state().y},
//                                                      agent.second.reference_line);
//      planning_msgs::TrajectoryPoint trajectory_point;
//      trajectory_point.relative_time =
//          static_cast<double>(i + 1) / static_cast<double>(kForwardSteps) * config_.sim_horizon_;
//      if (!this->SimulateOneStepForAgent(desired_velocity, agent.second, Agent(), trajectory_point)) {
//        return false;
//      }
//      agent.second.agent.MoveAgentToPoint(trajectory_point);
//
//      if (agent.second.agent.is_host()) {
//        ego_traj.trajectory_points.push_back(trajectory_point);
//      } else {
//        surrounding_trajs[agent.first].trajectory_points.push_back(trajectory_point);
//      }
//    }
//  }
//  return true;
//}

bool PolicyDecider::CloseLoopSimulate(const Policy &policy1,
                                      const std::unordered_map<int, SimulateAgent> &simulate_agents,
                                      planning_msgs::Trajectory &ego_traj,
                                      SurroundingTrajectories &surrounding_trajs) {
  const int kForwardSteps = static_cast<int>(config_.sim_horizon_ / config_.sim_step_);
  auto simulate_agent_tmp = simulate_agents;
  for (int i = 0; i < kForwardSteps; ++i) {
    for (auto &agent : simulate_agent_tmp) {
      auto behaviour = agent.second.behaviour_lane_pair.first;
      planning_msgs::TrajectoryPoint trajectory_point;
      if (behaviour == LateralBehaviour::UNDEFINED || agent.second.agent.agent_type() != AgentType::VEHICLE) {
        if (!NaivePredictionOnStepForAgent(agent.second, config_.sim_step_, trajectory_point)) {
          return false;
        }
      } else {
        auto reference_line = agent.second.behaviour_lane_pair.second;
        double desired_velocity =
            this->GetDesiredSpeed({agent.second.agent.state().x, agent.second.agent.state().y}, reference_line);
        int leading_agent_id = -1;
        if (!PolicyDecider::GetLeadingAgentOnRefLane(agent.second.agent,
                                                     simulate_agent_tmp,
                                                     reference_line,
                                                     leading_agent_id)) {
          return false;
        }
        // no leading agent or cannot find leading agent in simulate_agent_tmp
        if (leading_agent_id == -1 || simulate_agent_tmp.find(leading_agent_id) == simulate_agent_tmp.end()) {
          if (!this->SimulateOneStepForAgent(desired_velocity, agent.second, Agent(), trajectory_point)) {
            return false;
          }
        } else {
          auto leading_agent = simulate_agent_tmp[leading_agent_id];
          if (agent.second.agent.bounding_box().HasOverlapWithBox2d(leading_agent.agent.bounding_box())) {
            return false;
          }
          if (!this->SimulateOneStepForAgent(desired_velocity, agent.second, leading_agent.agent, trajectory_point)) {
            return false;
          }
        }
      }
      agent.second.agent.MoveAgentToPoint(trajectory_point);
      trajectory_point.relative_time =
          static_cast<double>(i + 1) / static_cast<double>(kForwardSteps) * config_.sim_horizon_;
      if (agent.second.agent.is_host()) {
        ego_traj.trajectory_points.push_back(trajectory_point);
      } else {
        surrounding_trajs[agent.first].trajectory_points.push_back(trajectory_point);
      }
    }
  }
  return true;
}

double PolicyDecider::GetDesiredSpeed(const std::pair<double, double> &xy, const ReferenceLine &ref_lane) const {
  auto ref_point = ref_lane.GetReferencePoint(xy);
  double kappa = ref_point.kappa();
  return std::min(config_.max_lat_acc / (std::fabs(kappa) + 1e-5), config_.desired_vel);
}

bool PolicyDecider::GetLeadingAgentOnRefLane(const Agent &agent,
                                             const std::unordered_map<int, SimulateAgent> &simulate_agents,
                                             const ReferenceLine &ref_lane,
                                             int &agent_id) {

  common::SLPoint ref_sl_point;
  if (!ref_lane.XYToSL({agent.state().x, agent.state().y}, &ref_sl_point)) {
    return false;
  }
  constexpr double kLatRange = 2.2;
  constexpr double kMaxForwardSearchDist = 100.0;
  constexpr double drive_buffer = 1.0;
  const double kResolution = kLatRange / 1.4;
  for (double s = ref_sl_point.s + kResolution;
       s < std::min(ref_lane.Length(), ref_sl_point.s + kMaxForwardSearchDist);
       s += kResolution) {
    const auto ref_point = ref_lane.GetReferencePoint(s);
    for (const auto &entry : simulate_agents) {
      if (entry.first == agent.id()) {
        continue;
      }
      if (std::hypot(entry.second.agent.state().x - ref_point.x(), entry.second.agent.state().y - ref_point.y())
          < kLatRange) {
        if (!ref_lane.IsBlockedByBox(entry.second.agent.bounding_box(), agent.bounding_box().width(), drive_buffer)) {
          continue;
        }
        agent_id = entry.first;
        return true;
      }
    }
  }
  return true;
}

bool PolicyDecider::SimulateOneStepForAgent(double desired_vel,
                                            const SimulateAgent &agent,
                                            const Agent &leading_agent,
                                            planning_msgs::TrajectoryPoint &trajectory_point) {
  if (!agent.agent.is_host() && agent.agent.is_static()) {
    return NaivePredictionOnStepForAgent(agent, config_.sim_step_, trajectory_point);
  }
//  if (agent.agent.agent_type() == AgentType::VEHICLE) {
  // agent is the vehicle
  SimulationParams simulation_params;
  simulation_params.idm_params.desired_velocity = desired_vel;
  //other params now is default;
  simulation_params.idm_params.acc_exponent = config_.acc_exponet;
  simulation_params.idm_params.max_acc = config_.max_acc;
  simulation_params.idm_params.max_decel = config_.max_decel;
  simulation_params.idm_params.safe_time_headway = config_.max_decel;
  simulation_params.idm_params.s0 = config_.s0;
  simulation_params.idm_params.s1 = config_.s1;
  simulation_params.default_lateral_approach_ratio = config_.default_lat_approach_ratio;
  simulation_params.cutting_in_lateral_approach_ratio = config_.cutting_in_lateral_approach_ratio;
  simulation_params.idm_params.safe_time_headway = config_.safe_time_headway;
  if (!leading_agent.is_valid()) {
    simulation_params.idm_params.leading_vehicle_length = agent.agent.bounding_box().length();
  } else {
    common::SLBoundary leading_sl_boundary;
    if (!agent.behaviour_lane_pair.second.GetSLBoundary(leading_agent.bounding_box(), &leading_sl_boundary)) {
      return false;
    }
//      simulation_params.idm_params.leading_vehicle_length = leading_agent.bounding_box().length();
    simulation_params.idm_params.leading_vehicle_length = leading_sl_boundary.end_s - leading_sl_boundary.start_s;
  }
  return forward_simulator_->ForwardOneStep(agent.agent,
                                            simulation_params,
                                            agent.behaviour_lane_pair.second,
                                            leading_agent,
                                            config_.sim_step_,
                                            trajectory_point);
}

bool PolicyDecider::NaivePredictionOnStepForAgent(const SimulateAgent &agent, double sim_step,
                                                  planning_msgs::TrajectoryPoint &trajectory_point) {
  if (!agent.agent.is_valid()) {
    return false;
  }
  auto next_state = agent.agent.state().GetNextStateAfterTime(sim_step);
  trajectory_point.path_point.x = next_state.x;
  trajectory_point.path_point.y = next_state.y;
  trajectory_point.path_point.theta = next_state.theta;
  trajectory_point.path_point.kappa = next_state.kappa;
  trajectory_point.path_point.dkappa = 0.0;
  trajectory_point.vel = next_state.v;
  trajectory_point.acc = next_state.a;
  trajectory_point.jerk = 0.0;
  return true;
}

bool PolicyDecider::EvaluateMultiPolicyTrajectories(const std::vector<Policy> &valid_policy,
                                                    const std::vector<planning_msgs::Trajectory> &valid_forward_trajectory,
                                                    const std::vector<SurroundingTrajectories> &valid_surrounding_trajectories,
                                                    PolicyDecider::Policy &winner_policy,
                                                    planning_msgs::Trajectory &winner_forward_trajectory,
                                                    double &winner_score) {
  if (valid_forward_trajectory.empty()) {
    return false;
  }
  if (valid_policy.size() != valid_forward_trajectory.size()) {
    return false;
  }
  if (valid_policy.size() != valid_surrounding_trajectories.size()) {
    return false;
  }
  double min_score = std::numeric_limits<double>::max();
  Policy policy;
  planning_msgs::Trajectory trajectory;
  for (size_t i = 0; i < valid_policy.size(); ++i) {
    double score = 0.0;
    EvaluateSinglePolicyTrajectory(valid_policy[i],
                                   valid_forward_trajectory[i],
                                   valid_surrounding_trajectories[i],
                                   &score);
    if (score < min_score) {
      min_score = score;
      policy = valid_policy.at(i);
      trajectory = valid_forward_trajectory[i];
    }
  }
  winner_forward_trajectory = trajectory;
  winner_policy = policy;
  winner_score = min_score;
  return true;
}

void PolicyDecider::EvaluateSinglePolicyTrajectory(const Policy &policy,
                                                   const planning_msgs::Trajectory &trajectory,
                                                   const SurroundingTrajectories &surrounding_trajs,
                                                   double *score) const {
  auto ref_lane = policy.second;
  // 1.efficiency
  const planning_msgs::TrajectoryPoint terminal_trajectory_point = trajectory.trajectory_points.back();
  const auto start_trajectory_point = trajectory.trajectory_points.front();
  double cost_efficiency_vel = std::fabs(terminal_trajectory_point.vel - config_.desired_vel) / 10.0;
  common::SLPoint start_sl, end_sl;
  ref_lane.XYToSL(terminal_trajectory_point.path_point.x, terminal_trajectory_point.path_point.y, &end_sl);
  ref_lane.XYToSL(start_trajectory_point.path_point.x, start_trajectory_point.path_point.y, &start_sl);
  double cost_efficiency_length = 2.0 / (std::fabs(end_sl.s - start_sl.s) + 1e-5);
  double cost_efficiency = cost_efficiency_length + cost_efficiency_vel;

  // 2. dead lane
  const double cost_dead_end = 1.0 / (std::fabs(ref_lane.Length() - start_sl.s) + 1e-5);

  //3. safety
  double cost_safety = 0.0;
  for (const auto &traj : surrounding_trajs) {
    double cost_safety_tmp = 0.0;
    if (traj.second.trajectory_points.empty()) {
      continue;
    }
    EvaluateSafetyCost(ego_id_, trajectory, traj, &cost_safety_tmp);
    cost_safety += cost_safety_tmp;
  }
  double cost_action = 0.0;
  if (policy.first != LateralBehaviour::LANE_KEEPING) {
    cost_action += 0.5;
  }
  *score = cost_action + cost_safety + cost_efficiency + cost_dead_end;
}

bool PolicyDecider::EvaluateSafetyCost(int ego_id, const planning_msgs::Trajectory &trajectory,
                                       const std::pair<int, planning_msgs::Trajectory> &other_trajectory,
                                       double *safety_cost) const {
  if (trajectory.trajectory_points.size() != other_trajectory.second.trajectory_points.size()) {
    return false;
  }
  auto ego_box = agent_set_.at(ego_id).bounding_box();
  const double length = ego_box.length();
  const double width = ego_box.width();
  auto other_box = agent_set_.at(other_trajectory.first).bounding_box();
  const double other_length = other_box.length();
  const double other_width = other_box.width();
  double cost_tmp = 0.0;
  for (size_t i = 0; i < trajectory.trajectory_points.size(); ++i) {
    const auto ego_path_point = trajectory.trajectory_points[i].path_point;
    const auto tmp_ego_box = common::Box2d({ego_path_point.x, ego_path_point.y},
                                           ego_path_point.theta, length, width);
    const auto other_path_point = other_trajectory.second.trajectory_points[i].path_point;
    const auto tmp_other_box = common::Box2d({other_path_point.x, other_path_point.y},
                                             other_path_point.theta, other_length, other_width);
    if (tmp_ego_box.HasOverlapWithBox2d(tmp_other_box)) {
      cost_tmp +=
          0.01 * fabs(trajectory.trajectory_points[i].vel - other_trajectory.second.trajectory_points[i].vel) *
              0.5;
    }
  }
  return true;
}

}