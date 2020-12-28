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
  std::vector<std::unordered_map<int, planning_msgs::Trajectory>> valid_surrounding_trajectories;

  for (const auto &policy : possible_policies) {
    planning_msgs::Trajectory trajectory;
    std::unordered_map<int, planning_msgs::Trajectory> surround_trajs;
    if (!SimulateEgoAgentPolicy(policy, trajectory, surround_trajs)) {
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
    return false;
  }
  best_policy = winner_policy;
  forward_policies = valid_policies;
  forward_trajectories = valid_trajectories;
  forward_surrounding_trajectories = valid_surrounding_trajectories;
  return true;
}

bool PolicyDecider::SimulateEgoAgentPolicy(const Policy &policy,
                                           planning_msgs::Trajectory &ego_traj,
                                           std::unordered_map<int, planning_msgs::Trajectory> &surrounding_trajs) {

  ego_traj.trajectory_points.clear();
  surrounding_trajs.clear();
  auto timestamp = ros::Time::now();
  planning_msgs::TrajectoryPoint init_trajectory_point;
  std::unordered_map<int, SimulateAgent> sim_agents;
  // 1. init trajectory point for each agent
  for (const auto &agent : agent_set_) {
    init_trajectory_point.path_point.x = agent.second.state().x;
    init_trajectory_point.path_point.y = agent.second.state().y;
    init_trajectory_point.path_point.kappa = agent.second.state().kappa;
    init_trajectory_point.path_point.dkappa = 0.0;
    init_trajectory_point.path_point.theta = agent.second.state().theta;
    init_trajectory_point.path_point.s = 0.0;
    init_trajectory_point.relative_time = 0.0;
    init_trajectory_point.vel = agent.second.state().v;
    init_trajectory_point.acc = agent.second.state().a;
    init_trajectory_point.jerk = 0.0;
    init_trajectory_point.relative_time = 0.0;
    SimulateAgent simulate_agent;
    simulate_agent.agent = agent.second;
    if (agent.second.is_host()) {
      simulate_agent.reference_line = policy.second;
      ego_traj.header.frame_id = "/map";
      ego_traj.header.stamp = timestamp;
      ego_traj.trajectory_points.push_back(init_trajectory_point);
    } else {
      simulate_agent.reference_line = agent.second.target_ref_lane();
      if (surrounding_trajs.find(agent.first) != surrounding_trajs.end()) {
        surrounding_trajs[agent.first].header.frame_id = "/map";
        surrounding_trajs[agent.first].header.stamp = timestamp;
        surrounding_trajs[agent.first].trajectory_points.push_back(init_trajectory_point);
      } else {
        planning_msgs::Trajectory traj;
        traj.header.stamp = timestamp;
        traj.header.frame_id = "/map";
        traj.trajectory_points.push_back(init_trajectory_point);
        surrounding_trajs.insert({agent.first, traj});
      }
    }
    sim_agents.insert({agent.first, simulate_agent});

  }
  if (!CloseLoopSimulate(policy, sim_agents, ego_traj, surrounding_trajs)) {
//    if (!OpenLoopSimForward(policy, sim_agents, ego_traj, surrounding_trajs)) {
//      return false;
//    }
    return false;
  }
  return true;
}

bool PolicyDecider::OpenLoopSimForward(const Policy &policy,
                                       const std::unordered_map<int, SimulateAgent> &simulate_agents,
                                       planning_msgs::Trajectory &ego_traj,
                                       std::unordered_map<int, planning_msgs::Trajectory> &surrounding_trajs) {

  const int kForwardSteps = static_cast<int>(config_.sim_horizon_ / config_.sim_step_);
  auto tmp_agent_set = simulate_agents;
  for (int i = 0; i < kForwardSteps; ++i) {

    for (auto &agent : tmp_agent_set) {
      double desired_velocity = this->GetDesiredSpeed({agent.second.agent.state().x, agent.second.agent.state().y},
                                                      *(agent.second.reference_line));
      planning_msgs::TrajectoryPoint trajectory_point;
      trajectory_point.relative_time =
          static_cast<double>(i + 1) / static_cast<double>(kForwardSteps) * config_.sim_horizon_;
      if (!this->SimulateOneStepForAgent(desired_velocity, agent.second, Agent(), trajectory_point)) {
        return false;
      }
      agent.second.agent.MoveAgentToPoint(trajectory_point);

      if (agent.second.agent.is_host()) {
        ego_traj.trajectory_points.push_back(trajectory_point);
      } else {
        surrounding_trajs[agent.first].trajectory_points.push_back(trajectory_point);
      }
    }
  }
  return true;
}

bool PolicyDecider::CloseLoopSimulate(const Policy &policy,
                                      const std::unordered_map<int, SimulateAgent> &simulate_agents,
                                      planning_msgs::Trajectory &ego_traj,
                                      std::unordered_map<int, planning_msgs::Trajectory> &surrounding_trajs) {
  const int kForwardSteps = static_cast<int>(config_.sim_horizon_ / config_.sim_step_);
  auto simulate_agent_tmp = simulate_agents;
  for (int i = 0; i < kForwardSteps; ++i) {
    for (auto &agent : simulate_agent_tmp) {
      double desired_velocity =
          this->GetDesiredSpeed({agent.second.agent.state().x, agent.second.agent.state().y},
                                *(agent.second.reference_line));

      planning_msgs::TrajectoryPoint trajectory_point;
      int leading_agent_id = -1;
      if (!PolicyDecider::GetLeadingAgentOnRefLane(agent.second.agent,
                                                   simulate_agent_tmp,
                                                   agent.second.reference_line,
                                                   leading_agent_id)) {
        return false;
      }

      // has no leading vehicle
      if (leading_agent_id == -1) {
        if (!this->SimulateOneStepForAgent(desired_velocity, agent.second, Agent(), trajectory_point)) {
          return false;
        }
      } else {
        auto leading_agent = simulate_agent_tmp[leading_agent_id].agent;
        // has leading agent but is overlapped with cur agent
        if (agent.second.agent.bounding_box().HasOverlapWithBox2d(leading_agent.bounding_box())) {
          return false;
        }
        if (!this->SimulateOneStepForAgent(desired_velocity,
                                           agent.second,
                                           leading_agent,
                                           trajectory_point)) {
          return false;
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
  return std::min(config_.max_lat_acc /(kappa + 1e-5), config_.desired_vel);
}

bool PolicyDecider::GetLeadingAgentOnRefLane(const Agent &agent,
                                             const std::unordered_map<int, SimulateAgent> &simulate_agents,
                                             const std::shared_ptr<ReferenceLine> &ref_lane,
                                             int &agent_id) {

  common::SLPoint ref_sl_point;
  if (!ref_lane->XYToSL({agent.state().x, agent.state().y}, &ref_sl_point)) {
    return false;
  }
  constexpr double kLatRange = 2.2;
  constexpr double kMaxForwardSearchDist = 100.0;
  constexpr double drive_buffer = 1.0;
  const double kResolution = kLatRange / 1.4;
  for (double s = ref_sl_point.s + kResolution; s < ref_sl_point.s + kMaxForwardSearchDist; s += kResolution) {
    const auto ref_point = ref_lane->GetReferencePoint(s);
    for (const auto &entry : simulate_agents) {
      if (entry.first == agent.id()) {
        continue;
      }
      if (std::hypot(entry.second.agent.state().x - ref_point.x(), entry.second.agent.state().y - ref_point.y())
          < kLatRange) {
        if (!ref_lane->IsBlockedByBox(entry.second.agent.bounding_box(), agent.bounding_box().width(), drive_buffer)) {
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
  if (agent.agent.agent_type() == AgentType::VEHICLE) {
    // agent is the vehicle
    SimulationParams simulation_params;
    simulation_params.idm_params.desired_velocity = desired_vel;
    //other params now is default;
    simulation_params.idm_params.acc_exponet = config_.acc_exponet;
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
      if (!agent.reference_line->GetSLBoundary(leading_agent.bounding_box(), &leading_sl_boundary)) {
        return false;
      }
//      simulation_params.idm_params.leading_vehicle_length = leading_agent.bounding_box().length();
      simulation_params.idm_params.leading_vehicle_length = leading_sl_boundary.end_s - leading_sl_boundary.start_s;
    }
    return forward_simulator_->ForwardOneStep(agent.agent,
                                              simulation_params,
                                              *(agent.reference_line),
                                              leading_agent,
                                              config_.sim_step_,
                                              trajectory_point);
  } else {
    // agent is not the vehicle, we use naive prediction trajectory.
    return NaivePredictionOnStepForAgent(agent, config_.sim_step_, trajectory_point);
  }
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
  // 1.efficiency
  const planning_msgs::TrajectoryPoint terminal_trajectory_point = trajectory.trajectory_points.back();
  double cost_efficiency = std::fabs(terminal_trajectory_point.vel - config_.desired_vel) / 10.0;

  //2.safety
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
  *score = cost_action + cost_safety + cost_efficiency;
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