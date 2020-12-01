#include "policy_decider.hpp"

namespace planning {
using SurroundingTrajectories = std::unordered_map<int, planning_msgs::Trajectory>;

PolicyDecider::PolicyDecider(const PolicySimulateConfig &config)
    : config_(config),
      forward_simulator_(std::make_unique<OnLaneForwardSimulator>()) {

}

bool PolicyDecider::PolicyDesicion(const Agent &ego_agent,
                                   const std::unordered_map<int, Agent> &other_agents,
                                   const PolicyDecider::Policies &policies,
                                   Behaviour &best_policy,
                                   double &desired_velocity) {

  std::vector<planning_msgs::Trajectory> valid_trajectories;
  std::vector<Policy> valid_policies;
  std::vector<std::unordered_map<int, planning_msgs::Trajectory>> valid_surrounding_trajectories;

  for (const auto &policy : policies) {
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
    init_trajectory_point.path_point.x = agent.second.state().x_;
    init_trajectory_point.path_point.y = agent.second.state().y_;
    init_trajectory_point.path_point.kappa = agent.second.state().kappa_;
    init_trajectory_point.path_point.dkappa = 0.0;
    init_trajectory_point.path_point.theta = agent.second.state().theta_;
    init_trajectory_point.path_point.s = 0.0;
    init_trajectory_point.relative_time = 0.0;
    init_trajectory_point.vel = agent.second.state().v_;
    init_trajectory_point.acc = agent.second.state().a_;
    init_trajectory_point.jerk = 0.0;
    init_trajectory_point.relative_time = 0.0;
    SimulateAgent simulate_agent;
    simulate_agent.agent = agent.second;
    sim_agents.insert({agent.first, simulate_agent});
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
  }

  if (!CloseLoopSimulate(policy, sim_agents, ego_traj, surrounding_trajs)) {
    if (!OpenLoopSimForward(policy, sim_agents, ego_traj, surrounding_trajs)) {
      return false;
    }
  }

}

bool PolicyDecider::OpenLoopSimForward(const Policy &policy,
                                       const std::unordered_map<int, SimulateAgent> &simulate_agents,
                                       planning_msgs::Trajectory &ego_traj,
                                       std::unordered_map<int, planning_msgs::Trajectory> &surrounding_trajs) {

  const int kForwardSteps = static_cast<int>(config_.sim_horizon_ / config_.sim_step_);
  for (int i = 0; i < kForwardSteps; ++i) {

    for (const auto &agent : simulate_agents) {
      double desired_velocity = this->GetDesiredSpeed({agent.second.agent.state().x_, agent.second.agent.state().y_},
                                                      *(agent.second.reference_line));
      planning_msgs::TrajectoryPoint trajectory_point;
      trajectory_point.relative_time =
          static_cast<double>(i + 1) / static_cast<double>(kForwardSteps) * config_.sim_horizon_;
      if (!this->SimulateOneStepForAgent(desired_velocity, agent.second, Agent(), trajectory_point)) {
        return false;
      }
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

bool PolicyDecider::CloseLoopSimulate(const Policy &policy,
                                      const std::unordered_map<int, SimulateAgent> &simulate_agents,
                                      planning_msgs::Trajectory &ego_traj,
                                      std::unordered_map<int, planning_msgs::Trajectory> &surrounding_trajs) {
  const int kForwardSteps = static_cast<int>(config_.sim_horizon_ / config_.sim_step_);
  for (int i = 0; i < kForwardSteps; ++i) {
    for (const auto &agent : simulate_agents) {
      double desired_velocity =
          this->GetDesiredSpeed({agent.second.agent.state().x_, agent.second.agent.state().y_},
                                *(agent.second.reference_line));

      planning_msgs::TrajectoryPoint trajectory_point;
      int leading_agent_id = -1;
      //has no leading agent
      if (!this->GetLeadingAgentOnRefLane(agent.second.agent, agent.second.reference_line, leading_agent_id)) {
        if (!this->SimulateOneStepForAgent(desired_velocity, agent.second, Agent(), trajectory_point)) {
          return false;
        }
      } else {
        auto leading_agent = agent_set_[leading_agent_id];
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
  return std::min(config_.max_lat_acc * kappa, config_.desired_vel);
}

bool PolicyDecider::GetLeadingAgentOnRefLane(const Agent &agent,
                                             const std::shared_ptr<ReferenceLine> &ref_lane,
                                             int &agent_id) const {

  common::SLPoint ref_sl_point;
  if (!ref_lane->XYToSL({agent.state().x_, agent.state().y_}, &ref_sl_point)) {
    return false;
  }
  constexpr double kLatRange = 2.2;
  constexpr double kMaxForwardSearchDist = 100.0;
  const double kResolution = kLatRange / 1.4;
  for (double s = ref_sl_point.s + kResolution;
       s < ref_sl_point.s + kMaxForwardSearchDist;
       s += kResolution) {
    const auto ref_point = ref_lane->GetReferencePoint(s);
    for (const auto &entry : agent_set_) {
      if (!entry.second.is_valid()) {
        continue;
      }
      if (std::hypot(entry.second.state().x_ - ref_point.x(), entry.second.state().y_ - ref_point.y()) < kLatRange) {
        agent_id = entry.first;
        return true;
      }
    }
  }

  return false;
}

bool PolicyDecider::SimulateOneStepForAgent(double desired_vel,
                                            const SimulateAgent &agent,
                                            const Agent &leading_agent,
                                            planning_msgs::TrajectoryPoint &trajectory_point) {
  SimulationParams simulation_params;
  simulation_params.idm_params.desired_velocity = desired_vel;
  //other params now is default;
  simulation_params.idm_params.acc_exponet = config_.acc_exponet;
  simulation_params.idm_params.max_decel = config_.max_decel;
  simulation_params.idm_params.safe_time_headway = config_.max_decel;
  simulation_params.idm_params.s0 = config_.s0;
  simulation_params.idm_params.s1 = config_.s1;
  simulation_params.max_default_lat_vel = config_.max_default_lat_vel;
  simulation_params.lat_vel_ratio = config_.lat_vel_ratio;
  simulation_params.idm_params.safe_time_headway = config_.safe_time_headway;
  simulation_params.lat_offset_threshold = config_.lat_offset_threshold;
  if (!leading_agent.is_valid()) {
    simulation_params.idm_params.leading_vehicle_length_ = agent.agent.bounding_box().length();
  } else {
    simulation_params.idm_params.leading_vehicle_length_ = leading_agent.bounding_box().length();
  }
  return forward_simulator_->ForwardOneStep(agent.agent,
                                            simulation_params,
                                            *(agent.reference_line),
                                            leading_agent,
                                            config_.sim_step_,
                                            trajectory_point);
}
bool PolicyDecider::EvaluateMultiPolicyTrajectories(const std::vector<Policy> &valid_policy,
                                                    const std::vector<planning_msgs::Trajectory> &valid_forward_trajectory,
                                                    const std::vector<SurroundingTrajectories> &valid_surrounding_trajectories,
                                                    PolicyDecider::Policy &winner_policy,
                                                    planning_msgs::Trajectory &winner_forward_trajectory,
                                                    double &winner_score) {

  return false;
}

}