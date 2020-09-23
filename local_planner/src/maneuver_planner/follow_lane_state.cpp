#include "maneuver_planner/emergency_stop_state.hpp"
#include "maneuver_planner/follow_lane_state.hpp"

#include <tf/transform_datatypes.h>

#include "obstacle_filter/obstacle_filter.hpp"
#include "reference_line/reference_line.hpp"
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
  this->TrafficLightsDecision(&traffic_maneuver_goal);
  this->ObstaclesDecision(&obstacle_maneuver_goal);
  return nullptr;
}

DecisionType FollowLaneState::TrafficLightsDecision(ManeuverGoal *maneuver_goal) const {
  bool prohibited = false;
  DecisionType decision_type;
  const double max_decel = PlanningConfig::Instance().max_acc();
  const double min_stop_distance = std::pow(VehicleState::Instance().linear_vel(), 2) / (2.0 * max_decel);
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
    decision_type = DecisionType::kEmergencyStop;

  } else if (min_dist > min_comfor_stop_distance) {
    prohibited = false;
  } else {
    prohibited = true;
    maneuver_goal->target_speed = 0.0;
    maneuver_goal->has_stop_point = true;
    maneuver_goal->target_s = std::max(nearest_traffic_light_sl_boundary.start_s, ego_s + min_stop_distance);
    maneuver_goal->lane_id = VehicleState::Instance().lane_id();
    maneuver_goal->decision_type = DecisionType::kStop;
    decision_type = DecisionType::kStop;
  }
  if (!prohibited) {
    maneuver_goal->target_speed = PlanningConfig::Instance().target_speed();
    maneuver_goal->has_stop_point = false;
    maneuver_goal->target_s = std::min(reference_line_->Length(),
                                       ego_s + PlanningConfig::Instance().max_lookahead_distance());
    maneuver_goal->lane_id = VehicleState::Instance().lane_id();
    maneuver_goal->decision_type = DecisionType::kFollowLane;
    decision_type = DecisionType::kFollowLane;
  }
  return decision_type;
}

DecisionType FollowLaneState::ObstaclesDecision(ManeuverGoal *maneuver_goal) const {
  const auto obstacles = ObstacleFilter::Instance().Obstacles();
  SLPoint ego_sl;
  const double half_ego_vehicle_length = PlanningConfig::Instance().vehicle_params().length / 2.0;
  const double ego_vel = VehicleState::Instance().linear_vel();
  const double max_acc = PlanningConfig::Instance().max_acc();
  const double min_stop_distance = ego_vel * ego_vel / (2.0 * max_acc);
  const double max_comfort_acc = max_acc * 0.6;
  const double min_comfort_stop_distance = ego_vel * ego_vel / (2.0 * max_comfort_acc);
  reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                          VehicleState::Instance().pose().position.y,
                          &ego_sl);
  double forward_clear_distance, backward_clear_distance;
  // 1.  check current lane
  SLBoundary ego_sl_boundary;
  int forward_obstacle_id, backward_obstacle_id;
  this->GetLaneClearDistance(0, &forward_clear_distance,
                       &backward_clear_distance,
                       &forward_obstacle_id,
                       &backward_obstacle_id);
  const auto &forward_obstacle = obstacles.at(forward_obstacle_id);
  if (forward_clear_distance > PlanningConfig::Instance().max_lookahead_distance()) {
    if (ego_sl.s + forward_clear_distance > reference_line_->Length()) {
      maneuver_goal->has_stop_point = true;
      maneuver_goal->target_s = reference_line_->Length();
      maneuver_goal->target_speed = 0.0;
      maneuver_goal->lane_id = VehicleState::Instance().lane_id();
      maneuver_goal->decision_type = DecisionType::kStop;
    } else {
      maneuver_goal->has_stop_point = false;
      maneuver_goal->target_s = ego_sl.s + forward_clear_distance;
      maneuver_goal->target_speed = PlanningConfig::Instance().target_speed();
      maneuver_goal->lane_id = VehicleState::Instance().lane_id();
      maneuver_goal->decision_type = DecisionType::kFollowLane;
    }
  } else {

  }

}

int FollowLaneState::SelectLane() const {

}

void FollowLaneState::GetLaneClearDistance(int lane_offset,
                                           double *const forward_clear_distance,
                                           double *const backward_clear_distance,
                                           int *const forward_obstacle_id,
                                           int *const backward_obstacle_id) const {
  double maneuver_forward_distance = PlanningConfig::Instance().max_lookahead_distance();
  double maneuver_backward_distance = PlanningConfig::Instance().max_lookback_distance();
  int front_obstacle_id = -1;
  int rear_obstacle_id = -1;
  SLBoundary ego_sl_boundary;
  reference_line_->GetSLBoundary(VehicleState::Instance().GetEgoBox(), &ego_sl_boundary);
  double forward_clear_dist = maneuver_forward_distance;
  double backward_clear_dist = maneuver_backward_distance;
  double left_lane_width, right_lane_width;
  double s = (ego_sl_boundary.start_s + ego_sl_boundary.end_s) * 0.5;
  reference_line_->GetLaneWidth(s, &left_lane_width, &right_lane_width);
  for (const auto &obstacle : ObstacleFilter::Instance().Obstacles()) {
    SLBoundary obstacle_sl_boundary;
    reference_line_->GetSLBoundary(obstacle.second->BoundingBox(), &obstacle_sl_boundary);
    if (obstacle_sl_boundary.start_s - ego_sl_boundary.end_s > maneuver_forward_distance) {
      continue;
    }
    if (obstacle_sl_boundary.end_s + maneuver_backward_distance < ego_sl_boundary.start_s) {
      continue;
    }
    switch (lane_offset) {
      case 0: {
        if (reference_line_->IsOnLane(obstacle_sl_boundary)) {
          double driving_width = reference_line_->GetDrivingWidth(obstacle_sl_boundary);
          if (driving_width
              < PlanningConfig::Instance().vehicle_params().width + PlanningConfig::Instance().safety_buffer()) {
            double forward_dist = obstacle_sl_boundary.start_s - ego_sl_boundary.end_s;
            if (forward_dist > 0.0 && forward_dist < forward_clear_dist) {
              forward_clear_dist = forward_dist;
              front_obstacle_id = obstacle.first;
            }
            double backward_dist = obstacle_sl_boundary.end_s - ego_sl_boundary.start_s;
            if (backward_dist < 0.0 && std::fabs(backward_dist) < backward_clear_dist) {
              backward_clear_dist = std::fabs(backward_dist);
              rear_obstacle_id = obstacle.first;
            }
          }
        }
        break;
      }
      case 1: {
        // right lane check
        if (obstacle_sl_boundary.end_l < 0 && obstacle_sl_boundary.start_l < -right_lane_width) {
          double forward_dist = obstacle_sl_boundary.start_s - ego_sl_boundary.end_s;
          double backward_dist = obstacle_sl_boundary.end_s - ego_sl_boundary.start_s;
          if (forward_dist > 0.0 && forward_dist < forward_clear_dist) {
            forward_clear_dist = forward_dist;
            front_obstacle_id = obstacle.first;
          }
          if (backward_dist < 0.0 && std::fabs(backward_dist) < backward_clear_dist) {
            backward_clear_dist = backward_dist;
            rear_obstacle_id = obstacle.first;
          }
        }
        break;
      }
      case -1: {
        // left lane check
        if (obstacle_sl_boundary.start_l > 0 && obstacle_sl_boundary.end_l > left_lane_width) {
          double forward_dist = obstacle_sl_boundary.start_s - ego_sl_boundary.end_s;
          double backward_dist = obstacle_sl_boundary.end_s - ego_sl_boundary.start_s;
          if (forward_dist > 0.0 && forward_dist < forward_clear_dist) {
            forward_clear_dist = forward_dist;
            front_obstacle_id = obstacle.first;
          }
          if (backward_dist < 0.0 && std::fabs(backward_dist) < backward_clear_dist) {
            backward_clear_dist = backward_dist;
            rear_obstacle_id = obstacle.first;
          }
        }
        break;
      }
      default:break;
    }
  }
  *forward_clear_distance = forward_clear_dist;
  *backward_clear_distance = backward_clear_dist;
  *forward_obstacle_id = front_obstacle_id;
  *backward_obstacle_id = rear_obstacle_id;
}
}
