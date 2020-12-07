#include "behaviour_planner.hpp"
#include "behaviour_strategy/mpdm_planner/mpdm_planner.hpp"
#include <memory>
#include <visualization_msgs/MarkerArray.h>
#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include "name/string_name.hpp"
#include "planning_msgs/Behaviour.h"

namespace planning {

BehaviourPlanner::BehaviourPlanner(const ros::NodeHandle &nh) : nh_(nh) {

  nh_.param<int>("/behaviour_planner/pool_size", pool_size_, 6);
  nh_.param<std::string>("/behaviour_planner/planner_type", planner_type_, "mpdm");
  nh_.param<double>("/behaviour_planner/desired_velocity", simulate_config_.desired_vel, 8.333);
  nh_.param<double>("/behaviour_planner/max_lat_acc", simulate_config_.max_lat_acc, 0.8);
  nh_.param<double>("/behaviour_planner/sim_horizon", simulate_config_.sim_horizon_, 10.0);
  nh_.param<double>("/behaviour_planner/sim_step", simulate_config_.sim_step_, 0.25);
  nh_.param<double>("/behaviour_planner/safe_time_headway", simulate_config_.safe_time_headway, 0.5);
  nh_.param<double>("/behaviour_planner/max_acc", simulate_config_.max_acc, 1.75);
  nh_.param<double>("/behaviour_planner/max_decel", simulate_config_.max_decel, 0.8);
  nh_.param<double>("/behaviour_planner/acc_exponet", simulate_config_.acc_exponet, 4.0);
  nh_.param<double>("/behaviour_planner/s0", simulate_config_.s0, 2.0);
  nh_.param<double>("/behaviour_planner/s1", simulate_config_.s1, 0.0);
  nh_.param<double>("/behaviour_planner/lateral_approach_ratio", simulate_config_.default_lat_approach_ratio, 0.995);
  nh_.param<double>("/behaviour_planner/cutting_in_lateral_approach_ratio",
                    simulate_config_.cutting_in_lateral_approach_ratio,
                    0.95);
  nh_.param<double>("/behaviour_planner/sample_lat_threshold", sample_key_agent_lat_threshold_, 6.0);
  nh_.param<double>("/behaviour_planner/sample_min_lon_threshold", sample_min_lon_threshold_, 20.0);
  thread_pool_ = std::make_unique<common::ThreadPool>(pool_size_);
  if (planner_type_ == "mpdm") {
    behaviour_strategy_ = std::make_unique<MPDMPlanner>(nh, simulate_config_, thread_pool_.get());
  } else {
    ROS_ERROR("No such type BehaviourPlanner, [%s]", planner_type_.c_str());
    ROS_ASSERT(false);
  }

  this->ego_vehicle_info_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleInfo>(
      common::topic::kEgoVehicleInfoName, 10,
      [this](const carla_msgs::CarlaEgoVehicleInfo::ConstPtr &ego_vehicle_info) {
        ego_vehicle_info_ = *ego_vehicle_info;
        this->ego_vehicle_id_ = ego_vehicle_info_.id;
        this->has_ego_vehicle_ = true;
        ROS_INFO("the ego_vehicle_id_: %i", ego_vehicle_id_);
      });
  this->traffic_light_info_subscriber_ =
      nh_.subscribe<carla_msgs::CarlaTrafficLightInfoList>(
          common::topic::kTrafficLightsInfoName, 10,
          [this](const carla_msgs::CarlaTrafficLightInfoList::ConstPtr &traffic_light_info_list) {
            this->traffic_light_info_list_ = *traffic_light_info_list;
          });
  this->traffic_light_status_subscriber_ = nh_.subscribe<carla_msgs::CarlaTrafficLightStatusList>(
      common::topic::kTrafficLigthsStatusName, 10,
      [this](const carla_msgs::CarlaTrafficLightStatusList::ConstPtr &traffic_light_status_list) {
        this->traffic_light_status_list_ = *traffic_light_status_list;
      }
  );
  this->ego_vehicle_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
      common::topic::kEgoVehicleStatusName, 10,
      [this](const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &ego_vehicle_status) {
        ego_vehicle_status_ = *ego_vehicle_status;
      });

  this->objects_subscriber_ = nh_.subscribe<derived_object_msgs::ObjectArray>(
      common::topic::kObjectsName, 10,
      [this](const derived_object_msgs::ObjectArray::ConstPtr &objects) {
//        for (const auto &object : objects->objects) {
//          agent_set_.emplace(object.id, object);
//        }
        this->objects_list_ = *objects;
      });

  this->behaviour_publisher_ = nh_.advertise<planning_msgs::Behaviour>(common::topic::kBehaviourName, 10);
  this->visualized_behaviour_trajectories_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>(common::topic::kVisualizedBehaviourTrajectoryName, 10);
}

void BehaviourPlanner::RunOnce() {
  if (behaviour_strategy_ == nullptr) {
    ROS_FATAL("BehaviourPlanner, Not Init BehaviourStrategy");
    return;
  }
  if (traffic_light_status_list_.traffic_lights.size() != traffic_light_info_list_.traffic_lights.size()) {
    return;
  }
  if (!MakeAgentSet()) {
    return;
  }
  if (!this->GetKeyAgents()) {
    return;
  }
  behaviour_strategy_->SetAgentSet(key_agent_set_);
  Behaviour behaviour;
  if (!behaviour_strategy_->Execute(behaviour)) {
    return;
  }
  planning_msgs::Behaviour publishable_behaviour;
  if (!BehaviourPlanner::ConvertBehaviourToRosMsg(behaviour, publishable_behaviour)) {
    return;
  }
  this->VisualizeBehaviourTrajectories(behaviour);
  behaviour_publisher_.publish(publishable_behaviour);

}

bool BehaviourPlanner::ConvertBehaviourToRosMsg(const Behaviour &behaviour,
                                                planning_msgs::Behaviour &behaviour_msg) {
  if (behaviour.forward_behaviours.empty()) {
    return false;
  }
  const size_t behaviour_size = behaviour.forward_behaviours.size();
  if (behaviour_size != behaviour.forward_trajs.size()) {
    return false;
  }
  if (behaviour_size != behaviour.surrounding_trajs.size()) {
    return false;
  }
  switch (behaviour.lateral_behaviour) {
    case LateralBehaviour::LANE_KEEPING:
      behaviour_msg.lat_behaviour.behaviour = planning_msgs::LateralBehaviour::LANEKEEPING;
      break;
    case LateralBehaviour::LANE_CHANGE_RIGHT:
      behaviour_msg.lat_behaviour.behaviour = planning_msgs::LateralBehaviour::LANECHANGERIGHT;
      break;
    case LateralBehaviour::LANE_CHANGE_LEFT:
      behaviour_msg.lat_behaviour.behaviour = planning_msgs::LateralBehaviour::LANECHANGELEFT;
      break;
    case LateralBehaviour::UNDEFINED:
    default:behaviour_msg.lat_behaviour.behaviour = planning_msgs::LateralBehaviour::UNDEFINED;
      break;
  }

  switch (behaviour.longitudinal_behaviour) {
    case LongitudinalBehaviour::MAINTAIN:
      behaviour_msg.lon_behaviour.behaviour = planning_msgs::LongitudinalBehaviour::MAINTAIN;
      break;
    case LongitudinalBehaviour::ACCELERATE:
      behaviour_msg.lon_behaviour.behaviour = planning_msgs::LongitudinalBehaviour::ACCELERATE;
      break;
    case LongitudinalBehaviour::DECELERATE:
      behaviour_msg.lon_behaviour.behaviour = planning_msgs::LongitudinalBehaviour::DECELERATE;
      break;
    case LongitudinalBehaviour::STOPPING:
    default:behaviour_msg.lon_behaviour.behaviour = planning_msgs::LongitudinalBehaviour::STOPPING;
      break;
  }

  behaviour_msg.forward_behaviours.resize(behaviour_size);
  behaviour_msg.forward_trajectories.resize(behaviour_size);
  behaviour_msg.surroud_trajectories.resize(behaviour_size);
  behaviour_msg.reference_lane.reserve(behaviour_size);
  for (size_t i = 0; i < behaviour_size; ++i) {
    switch (behaviour.forward_behaviours[i].first) {

      case LateralBehaviour::LANE_KEEPING:
        behaviour_msg.forward_behaviours[i].behaviour = planning_msgs::LateralBehaviour::LANEKEEPING;
        break;
      case LateralBehaviour::LANE_CHANGE_LEFT:
        behaviour_msg.forward_behaviours[i].behaviour = planning_msgs::LateralBehaviour::LANECHANGELEFT;
        break;
      case LateralBehaviour::LANE_CHANGE_RIGHT:
        behaviour_msg.forward_behaviours[i].behaviour = planning_msgs::LateralBehaviour::LANECHANGERIGHT;
        break;
      case LateralBehaviour::UNDEFINED:
      default:behaviour_msg.forward_behaviours[i].behaviour = planning_msgs::LateralBehaviour::UNDEFINED;
        break;
    }
    behaviour_msg.reference_lane[i].way_points = behaviour.forward_behaviours[i].second->way_points();
    behaviour_msg.forward_trajectories[i] = behaviour.forward_trajs[i];
    // for each agent
    behaviour_msg.surroud_trajectories[i].trajectories.reserve(behaviour.surrounding_trajs[i].size());
    for (const auto &item : behaviour.surrounding_trajs[i]) {
      planning_msgs::AgentTrajectory agent_trajectory;
      agent_trajectory.trajectory = item.second;
      agent_trajectory.id = item.first;
      behaviour_msg.surroud_trajectories[i].trajectories.emplace_back(agent_trajectory);
    }
  }
  return true;
}

bool BehaviourPlanner::GetKeyAgents() {
  if (!has_ego_vehicle_) {
    return false;
  }
  if (agent_set_.find(ego_vehicle_id_) == agent_set_.end()) {
    return false;
  }
  auto ego_agent = agent_set_[ego_vehicle_id_];
  const double front_distance =
      std::max(simulate_config_.desired_vel * simulate_config_.sim_horizon_, sample_min_lon_threshold_);
  const double back_distance = front_distance / 2.0;
  Eigen::Vector2d ego_heading{std::cos(ego_agent.state().theta_), std::sin(ego_agent.state().theta_)};
  key_agent_set_.insert({ego_vehicle_id_, agent_set_[ego_vehicle_id_]});
  key_agent_set_[ego_vehicle_id_].set_is_host(true);

  for (const auto &agent : agent_set_) {
    if (agent.first == ego_vehicle_id_) {
      continue;
    }
    Eigen::Vector2d agent_to_ego{agent.second.state().x_ - ego_agent.state().x_,
                                 agent.second.state().y_ - ego_agent.state().y_};

    const double cross_prod = agent_to_ego.x() * ego_heading.y() - agent_to_ego.y() * ego_heading.x();
    if (std::fabs(cross_prod) > sample_key_agent_lat_threshold_) {
      continue;
    }
    const double dot_prod = agent_to_ego.x() * ego_heading.x() + agent_to_ego.y() * ego_heading.y();
    if (dot_prod < front_distance && dot_prod > -back_distance) {
      key_agent_set_.insert(agent);
    }
  }
  return true;
}

void BehaviourPlanner::VisualizeBehaviourTrajectories(const Behaviour &behaviour) {
  visualization_msgs::MarkerArray marker_array;
  auto forward_trajectories = behaviour.forward_trajs;
  int i = 0;
  for (const auto &traj : forward_trajectories) {
    visualization_msgs::Marker points;
    points.type = visualization_msgs::Marker::POINTS;
    points.id = i;
    points.header.frame_id = "/map";
    points.action = visualization_msgs::Marker::ADD;
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.color.a = 1.0;
    points.color.b = 0.5;
    points.color.r = 0.4;
    for (const auto &tp : traj.trajectory_points) {
      geometry_msgs::Point point;
      point.x = tp.path_point.x;
      point.y = tp.path_point.y;
      point.z = 2.0;
      points.points.push_back(point);
    }
    i++;
  }
  visualized_behaviour_trajectories_publisher_.publish(marker_array);
}

bool BehaviourPlanner::MakeAgentSet() {
  agent_set_.clear();
  for (const auto &object : objects_list_.objects) {
    agent_set_.emplace(object.id, object);
  }
  for (size_t i = 0; i < traffic_light_info_list_.traffic_lights.size(); ++i) {
    if (traffic_light_status_list_.traffic_lights[i].state == carla_msgs::CarlaTrafficLightStatus::RED) {
      agent_set_.emplace(traffic_light_info_list_.traffic_lights[i].id,
                         Agent({traffic_light_status_list_.traffic_lights[i],
                                traffic_light_info_list_.traffic_lights[i]}));
    }

  }
  return true;
}

}

