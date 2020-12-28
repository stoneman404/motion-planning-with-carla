#include <memory>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include "name/string_name.hpp"
#include "planning_msgs/Behaviour.h"
#include "planning_srvs/AgentRouteService.h"
#include "behaviour_planner.hpp"
#include "behaviour_strategy/mpdm_planner/mpdm_planner.hpp"

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
  nh_.param<double>("/behaviour_planner/default_lateral_approach_ratio",
                    simulate_config_.default_lat_approach_ratio,
                    0.995);
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
  get_ego_vehicle_route_client_ =
      nh_.serviceClient<planning_srvs::RoutePlanService>(common::service::kRouteServiceName);
  get_agent_potential_routes_client_ =
      nh_.serviceClient<planning_srvs::AgentRouteService>(common::service::kGetAgentPotentialRouteServiceName);
  this->behaviour_publisher_ = nh_.advertise<planning_msgs::Behaviour>(common::topic::kBehaviourName, 10);
  this->visualized_behaviour_trajectories_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>(common::topic::kVisualizedBehaviourTrajectoryName, 10);
  this->visualized_agent_trajectories_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      common::topic::kVisualizedObstacleTrajectoriesName, 10);
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
  if (!this->PredictAgentsBehaviours()) {
    return;
  }
  Behaviour behaviour;
  if (!behaviour_strategy_->Execute(behaviour)) {
    ROS_FATAL("[BehaviourPlanner::RunOnce], failed tor excute.");
    return;
  }
  planning_msgs::Behaviour publishable_behaviour;
  if (!BehaviourPlanner::ConvertBehaviourToRosMsg(behaviour, publishable_behaviour)) {
    return;
  }
  behaviour_publisher_.publish(publishable_behaviour);
//  this->VisualizeBehaviourTrajectories(behaviour);
  VisualizeAgentTrajectories(behaviour);
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
  behaviour_msg.reference_lane.resize(behaviour_size);
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
//    behaviour_msg.reference_lanes(behaviour.forward_behaviours[i].second->way_points());
    planning_msgs::Lane lane;
    lane.way_points = behaviour.forward_behaviours[i].second->way_points();
    behaviour_msg.reference_lane[i] = lane;

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
  key_agent_set_.clear();
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
  Eigen::Vector2d ego_heading{std::cos(ego_agent.state().theta), std::sin(ego_agent.state().theta)};
  key_agent_set_.emplace(ego_vehicle_id_, ego_agent);
  key_agent_set_[ego_vehicle_id_].set_is_host(true);

  for (const auto &agent : agent_set_) {
    if (agent.first == ego_vehicle_id_) {
      continue;
    }
    Eigen::Vector2d agent_to_ego{agent.second.state().x - ego_agent.state().x,
                                 agent.second.state().y - ego_agent.state().y};

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
  int i = 0;
  for (const auto &traj :  behaviour.forward_trajs) {
    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.id = i;
    trajectory_marker.header.frame_id = "/map";
    trajectory_marker.action = visualization_msgs::Marker::ADD;
    trajectory_marker.scale.x = 0.2;
    trajectory_marker.scale.y = 0.2;
    trajectory_marker.color.a = 1.0;
    trajectory_marker.color.b = 0.5;
    trajectory_marker.color.r = 0.4;
    trajectory_marker.pose.orientation.w = 1.0;
    trajectory_marker.header.stamp = ros::Time::now();

    for (const auto &tp : traj.trajectory_points) {
      geometry_msgs::Point point;
      point.x = tp.path_point.x;
      point.y = tp.path_point.y;
      point.z = 2.0;
      trajectory_marker.points.emplace_back(point);
    }
    marker_array.markers.push_back(trajectory_marker);
    i++;
  }
  visualized_behaviour_trajectories_publisher_.publish(marker_array);
}

bool BehaviourPlanner::MakeAgentSet() {
  agent_set_.clear();
  for (const auto &object : objects_list_.objects) {
    agent_set_.emplace(object.id, object);
  }
//  for (size_t i = 0; i < traffic_light_info_list_.traffic_lights.size(); ++i) {
//    if (traffic_light_status_list_.traffic_lights[i].state == carla_msgs::CarlaTrafficLightStatus::RED) {
//      agent_set_.emplace(traffic_light_info_list_.traffic_lights[i].id,
//                         Agent({traffic_light_status_list_.traffic_lights[i],
//                                traffic_light_info_list_.traffic_lights[i]}));
//    }

//  }
  return true;
}
void BehaviourPlanner::VisualizeAgentTrajectories(const Behaviour &behaviour) {
  auto best_behaviour = behaviour.lateral_behaviour;
  size_t best_index = 0;
  for (size_t i = 0; i < behaviour.forward_behaviours.size(); ++i) {
    if (behaviour.forward_behaviours[i].first == best_behaviour) {
      best_index = i;
    }
  }
  auto best_agent_trajectories = behaviour.surrounding_trajs[best_index];
  visualization_msgs::MarkerArray marker_array;

  for (const auto &agent_trajectory : best_agent_trajectories) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.header.frame_id = "/map";
    marker.id = agent_trajectory.first;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.color.b = 0.6;
    marker.color.g = 0.8;
    marker.color.r = 0.4;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    for (const auto &tp : agent_trajectory.second.trajectory_points) {
      geometry_msgs::Point point;
      point.x = tp.path_point.x;
      point.y = tp.path_point.y;
      point.z = 2.0;
      marker.points.push_back(point);
    }
    marker_array.markers.push_back(marker);
  }
  this->visualized_agent_trajectories_publisher_.publish(marker_array);
}

bool BehaviourPlanner::PredictAgentsBehaviours() {
  if (key_agent_set_.empty()) {
    ROS_WARN("The key agent set is empty");
    return false;
  }
  for (auto &agent: key_agent_set_) {
    // if the agent is host, we dont assign probability.
    if (agent.second.is_host()) {
      continue;
    }
    if (agent.second.agent_type() != AgentType::VEHICLE) {
      agent.second.PredictAgentBehaviour(std::vector<std::shared_ptr<ReferenceLine>>(),
                                         simulate_config_.max_acc,
                                         -simulate_config_.max_decel,
                                         simulate_config_.sim_step_,
                                         simulate_config_.max_lat_acc,
                                         simulate_config_.desired_vel);
    } else {
      std::vector<std::shared_ptr<ReferenceLine>> agent_potential_lanes;
      if (!GetAgentPotentialRoutes(agent.first, &agent_potential_lanes)) {
        agent.second.PredictAgentBehaviour(std::vector<std::shared_ptr<ReferenceLine>>(),
                                           simulate_config_.max_acc,
                                           -simulate_config_.max_decel,
                                           simulate_config_.sim_step_,
                                           simulate_config_.max_lat_acc,
                                           simulate_config_.desired_vel);
      } else {
        agent.second.PredictAgentBehaviour(agent_potential_lanes,
                                           simulate_config_.max_acc,
                                           -simulate_config_.max_decel,
                                           simulate_config_.sim_step_,
                                           simulate_config_.max_lat_acc,
                                           simulate_config_.desired_vel);
      }
    }
  }
  return true;
}

bool BehaviourPlanner::GetAgentPotentialRoutes(int id, std::vector<std::shared_ptr<ReferenceLine>> *potential_lanes) {
  if (potential_lanes == nullptr) {
    return false;
  }
  planning_srvs::AgentRouteService srv;
  srv.request.actor_id = id;
  if (!get_agent_potential_routes_client_.call(srv)) {
    return false;
  }
  std::vector<planning_msgs::Lane> lane_arrays = srv.response.lanes;
  for (const auto &lane : lane_arrays) {
    std::shared_ptr<ReferenceLine> reference_line = std::make_shared<ReferenceLine>();
    if (!GenerateReferenceLine(agent_set_[id].state(),
                               lane, 60.0,
                               10.0,
                               false, potential_lanes)) {
      continue;
    }
  }
}

bool BehaviourPlanner::GenerateReferenceLine(const vehicle_state::KinoDynamicState &state,
                                             const planning_msgs::Lane &lane,
                                             double lookahead_length,
                                             double lookback_length,
                                             bool smooth,
                                             std::vector<std::shared_ptr<ReferenceLine>> *ptr_potential_lanes) {

  if (ptr_potential_lanes == nullptr || lane.way_points.empty()) {
    return false;
  }
  auto dist_sqr = [](const planning_msgs::WayPoint &way_point, Eigen::Vector2d &xy) -> double {
    return (way_point.pose.position.x - xy.x()) * (way_point.pose.position.x - xy.x())
        + (way_point.pose.position.y - xy.y()) * (way_point.pose.position.y - xy.y());
  };
  size_t index_min = 0;
  Eigen::Vector2d xy{state.x, state.y};
  double d_min = dist_sqr(lane.way_points.front(), xy);
  for (size_t i = 1; i < lane.way_points.size(); ++i) {
    double d = dist_sqr(lane.way_points[i], xy);
    if (d < d_min) {
      d_min = d;
      index_min = i;
    }
  }
  std::vector<planning_msgs::WayPoint> sampled_way_points;
  double s = 0;
  size_t index = index_min == 0 ? index_min : index_min - 1;;
  while (index > 0 || s > lookback_length - std::numeric_limits<double>::epsilon()) {
    sampled_way_points.push_back(lane.way_points[index]);
    Eigen::Vector2d xy_last{lane.way_points[index].pose.position.x, lane.way_points[index].pose.position.y};
    s += std::sqrt(dist_sqr(lane.way_points[--index], xy_last));
  }
  if (!sampled_way_points.empty()) {
    std::reverse(sampled_way_points.begin(), sampled_way_points.end());
  }
  index = index_min;
  s = 0;
  while (index < lane.way_points.size() || s > lookahead_length - std::numeric_limits<double>::epsilon()) {
    sampled_way_points.push_back(lane.way_points[index]);
    Eigen::Vector2d xy_last{lane.way_points[index].pose.position.x, lane.way_points[index].pose.position.y};
    s += std::sqrt(dist_sqr(lane.way_points[++index], xy_last));
  }
  if (sampled_way_points.size() < 3) {
    return false;
  }
  ptr_potential_lanes->emplace_back(std::make_shared<ReferenceLine>(sampled_way_points));
  return true;
}

bool BehaviourPlanner::GetEgoVehicleRoutes(const geometry_msgs::Pose &start_pose,
                                           const geometry_msgs::Pose &destination_pose) {
  return false;
}

}