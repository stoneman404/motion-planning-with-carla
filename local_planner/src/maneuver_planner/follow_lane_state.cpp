#include "maneuver_planner/emergency_stop_state.hpp"
#include "maneuver_planner/follow_lane_state.hpp"

#include <obstacle_filter/obstacle_filter.hpp>
#include <reference_line/reference_line.hpp>
#include <tf/transform_datatypes.h>
#include "planning_config.hpp"
#include "planning_context.hpp"
#include "planner/trajectory_planner.hpp"
#include "traffic_lights/traffic_light_list.hpp"
namespace planning {

bool FollowLaneState::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current switching to **FollowLaneState**...");
  const auto ego_pose = VehicleState::Instance().pose();
  const auto lane_id = VehicleState::Instance().lane_id();
  const auto goal_pose = PlanningContext::Instance().global_goal_pose().pose;
  planning_srvs::RouteResponse route_response;
  bool result = maneuver_planner->ReRoute(ego_pose, goal_pose, route_response);
  if (!result) {
    ROS_FATAL("Failed to enter **FollowLaneState** state");
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

bool FollowLaneState::Execute(ManeuverPlanner *maneuver_planner) {

  if (maneuver_planner == nullptr) {
    ROS_ERROR("the ManeuverPlanner is nullptr");
    return false;
  }

}

void FollowLaneState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are currently Exiting the FollowLaneState...");
}

State &FollowLaneState::Instance() {
  static FollowLaneState instance;
  return instance;
}

std::string FollowLaneState::Name() const { return "FollowLaneState"; }

State *FollowLaneState::NextState(ManeuverPlanner *maneuver_planner) const {

  if (maneuver_planner == nullptr) {
    ROS_ERROR("the ManeuverPlanner is nullptr");
    return nullptr;
  }
  ManeuverGoal traffic_maneuver_goal;
  ManeuverGoal obstacle_maneuver_goal;
  // traffic decision
  this->CurrentLaneIsProhibitedByTrafficLights(&traffic_maneuver_goal);
  this->CurrentLaneIsProhibitedByObstacles(&obstacle_maneuver_goal);
  return nullptr;
}

bool FollowLaneState::CurrentLaneIsProhibitedByTrafficLights(ManeuverGoal *maneuver_goal) const {
  bool prohibited = false;

  const double max_decel = PlanningConfig::Instance().max_acc();
  const double min_stop_distance = std::pow(VehicleState::Instance().linear_vel(), 2) / (2 * max_decel);
  const double max_comfort_decel = PlanningConfig::Instance().max_acc() * 0.6;
  double min_comfor_stop_distance = std::pow(VehicleState::Instance().linear_vel(), 2) / (2.0 * max_comfort_decel);
  min_comfor_stop_distance = std::max(min_comfor_stop_distance, 5.0);
  const int ego_lane_id = VehicleState::Instance().lane_id();
  const int ego_road_id = VehicleState::Instance().road_id();
  const auto traffic_lights = TrafficLightList::Instance().TrafficLights();
  const auto ego_box = VehicleState::Instance().GetEgoBox();
  SLBoundary ego_sl_boundary;
  reference_line_->GetSLBoundary(ego_box, &ego_sl_boundary);
  int min_id = -1;
  double min_dist = std::numeric_limits<double>::max();
  SLBoundary nearest_traffic_light_sl_boundary;
  for (const auto &traffic_light : traffic_lights) {
    if (traffic_light.second->RoadId() != ego_road_id) {
      continue;
    }
    if (traffic_light.second->LaneId() != ego_lane_id) {
      continue;
    }
    SLBoundary sl_boundary;
    const Box2d light_box = traffic_light.second->GetBox2d();
    reference_line_->GetSLBoundary(light_box, &sl_boundary);
    if (sl_boundary.end_s < ego_sl_boundary.start_s) {
      continue;
    }

    double dist = sl_boundary.start_s - ego_sl_boundary.end_s;
    if (dist < min_dist) {
      min_dist = dist;
      min_id = traffic_light.first;
      nearest_traffic_light_sl_boundary = sl_boundary;
    }
  }
  const auto traffic_state = traffic_lights.at(min_id)->TrafficLightStatus().state;
  double ego_s = 0.5 * (ego_sl_boundary.start_s + ego_sl_boundary.end_s);

  if (traffic_state == carla_msgs::CarlaTrafficLightStatus::GREEN
      || traffic_state == carla_msgs::CarlaTrafficLightStatus::OFF
      || traffic_state == carla_msgs::CarlaTrafficLightStatus::UNKNOWN) {
    prohibited = false;
  } else if (min_dist < min_stop_distance) {
    prohibited = true;
    maneuver_goal->has_stop_point = true;
    maneuver_goal->decision_type = DecisionType::kEmergencyStop;
    maneuver_goal->lane_id = VehicleState::Instance().lane_id();
    maneuver_goal->target_s = std::max(nearest_traffic_light_sl_boundary.start_s, ego_s + min_stop_distance);
    maneuver_goal->target_speed = 0.0;

  } else if (min_dist > min_comfor_stop_distance) {
    prohibited = false;
  } else {
    prohibited = true;
    maneuver_goal->target_speed = 0.0;
    maneuver_goal->has_stop_point = true;
    maneuver_goal->target_s = std::max(nearest_traffic_light_sl_boundary.start_s, ego_s + min_stop_distance);
    maneuver_goal->lane_id = VehicleState::Instance().lane_id();
    maneuver_goal->decision_type = DecisionType::kStop;
  }
  if (!prohibited) {
    maneuver_goal->target_speed = PlanningConfig::Instance().target_speed();
    maneuver_goal->has_stop_point = false;
    maneuver_goal->target_s = std::min(reference_line_->Length(),
                                       ego_s + PlanningConfig::Instance().max_lookahead_distance());
    maneuver_goal->lane_id = VehicleState::Instance().lane_id();
    maneuver_goal->decision_type = DecisionType::kFollowLane;
  }


  return prohibited;
}

bool FollowLaneState::CurrentLaneIsProhibitedByObstacles(ManeuverGoal* maneuver_goal) const {
  const int current_lane_id = VehicleState::Instance().lane_id();
  const int current_road_id = VehicleState::Instance().road_id();
  auto obstacles = ObstacleFilter::Instance().Obstacles();
  const double ego_vel = VehicleState::Instance().linear_vel();

  const double lookahead_distance = std::max(
      PlanningConfig::Instance().max_lookahead_distance(),
      PlanningConfig::Instance().max_lookahead_time() * ego_vel);
  SLBoundary ego_sl_boundary;
  reference_line_->GetSLBoundary(VehicleState::Instance().GetEgoBox(), &ego_sl_boundary);
  bool prohibited = false;
  double min_ds = std::numeric_limits<double>::max();
  int min_obstacle_id = -1;
  SLBoundary nearest_sl_boundary;
  // get nearest front obstacles
  for (const auto &obstacle : obstacles) {
    if (obstacle.second->road_id() != current_road_id) {
      continue;
    }
    SLBoundary sl_boundary;
    reference_line_->GetSLBoundary(obstacle.second->BoundingBox(), &sl_boundary);
    if (!reference_line_->IsOnLane(sl_boundary)) {
      continue;
    }
    if (ego_sl_boundary.start_s > sl_boundary.end_s + 1e-1) {
      continue;
    }
    if (ego_sl_boundary.end_s
        + lookahead_distance < sl_boundary.start_s) {
      continue;
    }
    double dist = ego_sl_boundary.start_s - ego_sl_boundary.end_s;
    if (dist < min_ds) {
      min_ds = dist;
      min_obstacle_id = obstacle.first;
      nearest_sl_boundary = sl_boundary;
    }
  }
  double ego_s = 0.5 * (ego_sl_boundary.start_s + ego_sl_boundary.end_s);

  if (min_obstacle_id == -1) {
    prohibited = false;
    maneuver_goal->target_s = std::min(ego_s + lookahead_distance, reference_line_->Length());
    maneuver_goal->has_stop_point = false;
    maneuver_goal->target_speed = PlanningConfig::Instance().target_speed();
    maneuver_goal->lane_id = VehicleState::Instance().lane_id();
    maneuver_goal->decision_type = DecisionType::kFollowLane;
  } else {

  }

  return prohibited;
}

bool FollowLaneState::WithInDistanceAhead(double target_x,
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
  double d_angle = MathUtil::NormalizeAngle(
      std::acos(forward_vector.dot(target_vector)));
  return d_angle < M_PI / 2.0;
}
}
