#include "maneuver_planner/emergency_stop_state.hpp"
#include "maneuver_planner/keep_lane_state.hpp"

#include <obstacle_filter/obstacle_filter.hpp>
#include <reference_line/reference_line.hpp>
#include <tf/transform_datatypes.h>
#include "planning_config.hpp"
#include "planning_context.hpp"
#include "planner/trajectory_planner.hpp"
#include "traffic_lights/traffic_light_list.hpp"
namespace planning {

bool KeepLaneState::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current switching to **KeepLaneState**...");
  const auto ego_pose = VehicleState::Instance().pose();
  const auto lane_id = VehicleState::Instance().lane_id();
  const auto goal_pose = PlanningContext::Instance().global_goal_pose().pose;
  planning_srvs::RouteResponse route_response;
  bool result = maneuver_planner->ReRoute(ego_pose, goal_pose, route_response);
  if (!result) {
    ROS_FATAL("Failed to enter **KeepLaneState** state");
    return false;
  }
  auto &route_infos = PlanningContext::Instance().mutable_route_infos();
  if (!route_infos.empty()) {
    route_infos.clear();
  }
  route_infos.push_back(route_response);
  std::list<std::shared_ptr<ReferenceLine>> reference_lines;
  bool reference_result = maneuver_planner->UpdateReferenceLine(&reference_lines);

  if (!reference_result) {
    return false;
  }
  auto &reference_line_list = PlanningContext::Instance().mutable_reference_lines();
  reference_line_list.clear();
  reference_line_list = reference_lines;
  ROS_ASSERT(reference_line_list.size() == 1);
}

bool KeepLaneState::Execute(ManeuverPlanner *maneuver_planner) {

  if (maneuver_planner == nullptr) {
    ROS_ERROR("the ManeuverPlanner is nullptr");
    return false;
  }
}

void KeepLaneState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are currently Exiting the KeepLaneState...");
}

State &KeepLaneState::Instance() {
  static KeepLaneState instance;
  return instance;
}

std::string KeepLaneState::Name() const { return "KeepLaneState"; }

State *KeepLaneState::NextState(ManeuverPlanner *maneuver_planner) const {

  if (maneuver_planner == nullptr) {
    ROS_ERROR("the ManeuverPlanner is nullptr");
    return nullptr;
  }
  auto next_states = GetPosibileNextStates();
  return nullptr;
}

std::vector<StateName> KeepLaneState::GetPosibileNextStates() const {
  std::vector<StateName> possible_states;
  possible_states.push_back(StateName::kEmergencyStop);
  possible_states.push_back(StateName::kChangeLaneLeft);
  possible_states.push_back(StateName::kChangeLaneRight);
  possible_states.push_back(StateName::kKeepLane);
  possible_states.push_back(StateName::kStopAtSign);
  return possible_states;
}

bool KeepLaneState::CurrentLaneProhibitedByTrafficLight() const {
  bool prohibited = false;
  double max_comfort_decel = PlanningConfig::Instance().max_acc() * 0.6;  // todo: 0.6 should be parameter
  double comfort_stop_distance = std::pow(VehicleState::Instance().linear_vel(), 2); // max_comfort_decel

  const int ego_lane_id = VehicleState::Instance().lane_id();
  const int ego_road_id = VehicleState::Instance().road_id();
  const auto traffic_lights = TrafficLightList::Instance().TrafficLights();
  SLPoint ego_sl;
  reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                          VehicleState::Instance().pose().position.y, &ego_sl);
  int min_id = -1;
  double min_dist = std::numeric_limits<double>::max();
  // get nearest front traffic light in same lane
  for (const auto &traffic_light : traffic_lights) {
    if (traffic_light.second.RoadId() != ego_road_id) {
      continue;
    }
    if (traffic_light.second.LaneId() != ego_lane_id) {
      continue;
    }
    SLPoint sl_point;
    reference_line_->XYToSL(traffic_light.second.WayPoint().pose.position.x,
                            traffic_light.second.WayPoint().pose.position.y,
                            &sl_point);
    if (sl_point.s < ego_sl.s) {
      continue;
    }
    if (std::fabs(sl_point.s - ego_sl.s) < min_dist) {
      min_dist = std::fabs(sl_point.s - ego_sl.s);
      min_id = traffic_light.first;
    }
  }
  const auto traffic_state = traffic_lights.at(min_id).TrafficLightStatus().state;
  const auto trigger_volume = traffic_lights.at(min_id).TrafficLightBoundingBox();

  return prohibited;
}

bool KeepLaneState::CurrentLaneProhibitedByObstacles() const {
  const int current_lane_id = VehicleState::Instance().lane_id();
  const int current_road_id = VehicleState::Instance().road_id();
  auto obstacles = ObstacleFilter::Instance().Obstacles();
  const double ego_vel = VehicleState::Instance().linear_vel();

  const double ego_width = PlanningConfig::Instance().vehicle_params().width;
  const double safe_buffer = 1.0;
  const double lookahead_distance = std::max(
      PlanningConfig::Instance().max_lookahead_distance(),
      PlanningConfig::Instance().max_lookahead_time() * ego_vel);
  SLPoint ego_sl;
  reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                          VehicleState::Instance().pose().position.y, &ego_sl);

  bool prohibited = false;
  double min_ds = std::numeric_limits<double>::max();
  int min_obstacle_id = 0;
  // get nearest front obstacles
  for (const auto &obstacle : obstacles) {
    if (obstacle.second->road_id() != current_road_id) {
      continue;
    }
    if (obstacle.second->lane_id() != current_lane_id) {
      continue;
    }
    SLPoint sl_point;
    reference_line_->XYToSL(obstacle.second->Center().x(),
                            obstacle.second->Center().y(), &sl_point);
    if (sl_point.s < ego_sl.s) {
      continue;
    }

    if (sl_point.s - ego_sl.s < min_ds) {
      min_obstacle_id = obstacle.first;
    }
  }

  const auto nearest_obstacle = obstacles[min_obstacle_id];
  if (nearest_obstacle->IsStatic()) {
    SLBoundary sl_boundary;
    reference_line_->GetSLBoundary(nearest_obstacle->BoundingBox(), &sl_boundary);
    if (!reference_line_->IsOnLane(sl_boundary)) {
      prohibited = false;
    } else if (sl_boundary.start_s < ego_sl.s
        || sl_boundary.end_s > ego_sl.s + lookahead_distance) {
      prohibited = false;
    } else {
      prohibited = reference_line_->GetDrivingWidth(sl_boundary) <= ego_width + safe_buffer;
    }
  } else {
    // for dynamic obstacles
    prohibited = nearest_obstacle->Speed() < 0.3 * ego_vel;
  }
  return prohibited;
}

bool KeepLaneState::WithInDistanceAhead(double target_x,
                                        double target_y,
                                        double current_x,
                                        double current_y,
                                        double heading,
                                        double max_distance) {
  Eigen::Vector2d target_vector;
  target_vector << target_x - current_x, target_y - current_y;
  double norm_target = target_vector.norm();
  if (norm_target < 0.0) {
    return true;
  }
  if (norm_target > max_distance) {
    return false;
  }
  Eigen::Vector2d forward_vector;
  forward_vector << std::cos(heading), std::sin(heading);
  double d_angle = NormalizeAngle(
      std::acos(forward_vector.dot(target_vector)));
  return d_angle < M_PI / 2.0;
}
}
