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

void FollowLaneState::TrafficLightsDecision(ManeuverGoal *maneuver_goal) const {
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
}

void FollowLaneState::ObstaclesDecision(ManeuverGoal *maneuver_goal) const {
  const auto obstacles = ObstacleFilter::Instance().Obstacles();
  SLPoint ego_sl;
  const double half_ego_vehicle_length = PlanningConfig::Instance().vehicle_params().length / 2.0;
  const double ego_vel = VehicleState::Instance().linear_vel();
  const double max_acc = PlanningConfig::Instance().max_acc();
  reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                          VehicleState::Instance().pose().position.y,
                          &ego_sl);
  double forward_clear_distance, backward_clear_distance;
  //  check current lane
  int forward_obstacle_id, backward_obstacle_id;
  this->GetLaneClearDistance(0, &forward_clear_distance,
                             &backward_clear_distance,
                             &forward_obstacle_id,
                             &backward_obstacle_id);

  // determine the road option for vehicle
  size_t nearest_index;
  size_t maneuver_lookahead_steps = 10;
  const auto &way_points = reference_line_->way_points();
  auto nearest_way_point = reference_line_->NearestWayPoint(VehicleState::Instance().pose().position.x,
                                                            VehicleState::Instance().pose().position.y,
                                                            &nearest_index);
  size_t incoming_way_point_index = std::min(nearest_index + maneuver_lookahead_steps, way_points.size() - 1);
  auto incoming_way_point = way_points[incoming_way_point_index];
  double lookahead_distance =
      std::min(std::max(forward_clear_distance - PlanningConfig::Instance().lon_safety_buffer(),
                        PlanningConfig::Instance().min_lookahead_distance()),
               PlanningConfig::Instance().max_lookahead_distance());
  const double reference_line_length = reference_line_->Length();
  // 0. no leading vehicle
  if (forward_obstacle_id < 0) {
    // 0.1 no leading vehicle and near the reference line's end
    if (ego_sl.s + lookahead_distance > reference_line_length) {
      maneuver_goal->target_s = reference_line_->Length();
      maneuver_goal->target_speed = 0.0;
      maneuver_goal->has_stop_point = true;
      maneuver_goal->lane_id = incoming_way_point.lane_id;
      maneuver_goal->decision_type = DecisionType::kStop;
    } else {
      // 0.2 no leading vehicle and not near the reference line's end
      maneuver_goal->target_speed = PlanningConfig::Instance().target_speed();
      maneuver_goal->target_s = ego_sl.s + lookahead_distance;
      maneuver_goal->lane_id = incoming_way_point.lane_id;
      maneuver_goal->has_stop_point = false;
      maneuver_goal->decision_type = DecisionType::kFollowLane;
    }
    return;
  }
  const auto &forward_obstacle = obstacles.at(forward_obstacle_id);
  // 1. if there is a vehicle is so close
  const double vel_diff = forward_obstacle->Speed() - ego_vel;
  const double min_stop_distance = vel_diff * vel_diff / (2.0 * max_acc);
  const double min_comfort_stop_distance = vel_diff * vel_diff / (2.0 * max_acc * 0.6);
  if (forward_clear_distance < PlanningConfig::Instance().lon_safety_buffer()) {
    maneuver_goal->target_s = ego_sl.s + forward_clear_distance;
    maneuver_goal->target_speed = 0.0;
    maneuver_goal->lane_id = incoming_way_point.lane_id;
    maneuver_goal->has_stop_point = true;
    maneuver_goal->decision_type = DecisionType::kEmergencyStop;
    return;
  }

  // 2. has leading vehicle

  // 2.1 leading vehicle is far away the ego vehicle
  if (lookahead_distance > PlanningConfig::Instance().max_lookahead_distance()) {
    // 2.1.1 has stop point
    if (ego_sl.s + lookahead_distance > reference_line_length) {
      maneuver_goal->has_stop_point = true;
      maneuver_goal->target_s = reference_line_->Length();
      maneuver_goal->target_speed = 0.0;
      maneuver_goal->lane_id = VehicleState::Instance().lane_id();
      maneuver_goal->decision_type = DecisionType::kStop;
    } else {
      // 2.1.2 does not have stop point
      maneuver_goal->has_stop_point = false;
      maneuver_goal->target_s = ego_sl.s + lookahead_distance;
      maneuver_goal->target_speed = PlanningConfig::Instance().target_speed();
      maneuver_goal->lane_id = VehicleState::Instance().lane_id();
      maneuver_goal->decision_type = DecisionType::kFollowLane;
    }
  } else {
    // 2.2 leading vehicle is in lookahead distance
    if (ego_sl.s + lookahead_distance < reference_line_length) {
      // 2.2.1 check to left and right lane
      if (0.3 * ego_vel > forward_obstacle->Speed()) {
        if (incoming_way_point.road_option.option == planning_msgs::CarlaRoadOption::LANEFOLLOW) {
          // todo: change lane decision

        } else {
          // follow the leading vehicle
          maneuver_goal->decision_type = DecisionType::kFollowLane;
          maneuver_goal->lane_id = incoming_way_point.lane_id;
          maneuver_goal->target_s = std::min(ego_sl.s + lookahead_distance, reference_line_->Length());
          maneuver_goal->has_stop_point = false;
          maneuver_goal->target_speed = std::min(forward_obstacle->Speed(), PlanningConfig::Instance().target_speed());
        }
      } else {
        // 2.2.2 the leading vehicle is not so slow, just follow
        maneuver_goal->decision_type = DecisionType::kFollowLane;
        maneuver_goal->lane_id = incoming_way_point.lane_id;
        maneuver_goal->target_s = std::min(ego_sl.s + lookahead_distance, reference_line_length);
        maneuver_goal->has_stop_point = false;
        maneuver_goal->target_speed = std::min(forward_obstacle->Speed(), PlanningConfig::Instance().target_speed());
      }
    } else {
      // 2.3 the vehicle is approach the reference end, we don't want to change the lane
      maneuver_goal->has_stop_point = true;
      maneuver_goal->lane_id = incoming_way_point.lane_id;
      maneuver_goal->target_speed = 0.0;
      maneuver_goal->target_s = reference_line_length;
      maneuver_goal->decision_type = DecisionType::kStop;
    }
  }
}

void FollowLaneState::ChangeLaneDecision(double ego_s,
                                         double current_lane_forward_clear_distance,
                                         double current_lane_backward_clear_distance,
                                         int current_lane_forward_obstacle_id,
                                         int current_lane_backward_obstacle_id,
                                         const planning_msgs::WayPoint &incoming_way_point,
                                         ManeuverGoal *maneuver_goal) const {
  double left_forward_clear_distance = -10.0;
  double left_backward_clear_distance = -10.0;
  int left_forward_obstacle_id;
  int left_backward_obstacle_id;
  double right_forward_clear_distance = -10.0;
  double right_backward_clear_distance = -10.0;
  int right_forward_obstacle_id;
  int right_backward_obstacle_id;
  bool can_change_left = false;
  bool can_change_right = false;
  const auto &rear_obstacle = ObstacleFilter::Instance().Obstacles().at(current_lane_backward_obstacle_id);
  const auto &front_obstacle = ObstacleFilter::Instance().Obstacles().at(current_lane_forward_obstacle_id);
  // check current lane is clear to change lane
  if (current_lane_forward_clear_distance < PlanningConfig::Instance().maneuver_forward_clear_threshold()
      || current_lane_backward_clear_distance < PlanningConfig::Instance().maneuver_backward_clear_threshold()) {
    // current lane is unsafe to change lane, because leading or follow vehicle is so closed
    can_change_right = false;
    can_change_left = false;
  } else {
    // current lane is safe to change lane, check left lane
    if ((incoming_way_point.lane_change.type == planning_msgs::LaneChangeType::LEFT ||
        incoming_way_point.lane_change.type == planning_msgs::LaneChangeType::BOTH) &&
        incoming_way_point.has_left_lane) {
      this->GetLaneClearDistance(-1,
                                 &left_forward_clear_distance,
                                 &left_backward_clear_distance,
                                 &left_forward_obstacle_id,
                                 &left_backward_obstacle_id);
      if (left_backward_clear_distance > PlanningConfig::Instance().maneuver_target_lane_backward_clear_threshold()
          && left_forward_clear_distance > PlanningConfig::Instance().maneuver_target_lane_forward_clear_threshold()) {
        if (left_forward_obstacle_id < 0) {
          can_change_left = true;
        } else {
          const double ego_vel = VehicleState::Instance().linear_vel();
          const auto &left_leading_obstacle = ObstacleFilter::Instance().Obstacles().at(left_forward_obstacle_id);
          const double leading_vel = left_leading_obstacle->Speed();
          if (left_backward_obstacle_id < 0) {
            can_change_left = leading_vel > std::min(ego_vel, PlanningConfig::Instance().target_speed());
          } else {
            const auto &left_backward_obstacle = ObstacleFilter::Instance().Obstacles().at(left_backward_obstacle_id);
            const double follow_vel = left_backward_obstacle->Speed();
            can_change_left = follow_vel < std::min(PlanningConfig::Instance().target_speed(), leading_vel);
          }
        }
      }
    } else {
      can_change_left = false;
    }
    // check right lane
    if ((incoming_way_point.lane_change.type == planning_msgs::LaneChangeType::RIGHT ||
        incoming_way_point.lane_change.type == planning_msgs::LaneChangeType::BOTH) &&
        incoming_way_point.has_right_lane) {

      this->GetLaneClearDistance(1,
                                 &right_forward_clear_distance,
                                 &right_backward_clear_distance,
                                 &right_forward_obstacle_id,
                                 &right_backward_obstacle_id);
      if (right_backward_clear_distance > PlanningConfig::Instance().maneuver_target_lane_backward_clear_threshold()
          && right_forward_clear_distance > PlanningConfig::Instance().maneuver_target_lane_forward_clear_threshold()) {
        if (right_forward_obstacle_id < 0) {
          can_change_right = true;
        } else {
          const double ego_vel = VehicleState::Instance().linear_vel();
          const auto &right_leading_obstacle = ObstacleFilter::Instance().Obstacles().at(right_forward_obstacle_id);
          const double leading_vel = right_leading_obstacle->Speed();
          if (right_backward_obstacle_id < 0) {
            can_change_right = leading_vel > std::min(ego_vel, PlanningConfig::Instance().target_speed());
          } else {
            const auto &right_backward_obstacle = ObstacleFilter::Instance().Obstacles().at(right_backward_obstacle_id);
            const double follow_vel = right_backward_obstacle->Speed();
            can_change_right = follow_vel < std::min(PlanningConfig::Instance().target_speed(), leading_vel);
          }
        }
      }
    } else {
      can_change_right = false;
    }
  }
  if (!can_change_right && !can_change_left) {
    const double lookahead_distance = std::max(PlanningConfig::Instance().min_lookahead_distance(),
                                               std::min(PlanningConfig::Instance().max_lookahead_distance(),
                                                        current_lane_forward_clear_distance
                                                            - PlanningConfig::Instance().lon_safety_buffer()));
    maneuver_goal->decision_type = DecisionType::kFollowLane;
    maneuver_goal->lane_id = incoming_way_point.lane_id;
    maneuver_goal->target_speed = std::min(PlanningConfig::Instance().target_speed(),
                                           ObstacleFilter::Instance().Obstacles().at(current_lane_forward_obstacle_id)->Speed());
    maneuver_goal->has_stop_point = false;
    maneuver_goal->target_s = ego_s + lookahead_distance;
  } else if (can_change_left && !can_change_right) {
    // only can change left
    const double lookahead_distance = std::max(PlanningConfig::Instance().min_lookahead_distance(),
                                               std::min(PlanningConfig::Instance().max_lookahead_distance(),
                                                        left_forward_clear_distance
                                                            - PlanningConfig::Instance().lon_safety_buffer()));
    maneuver_goal->decision_type = DecisionType::kChangeLeft;
    maneuver_goal->target_speed = std::min(ObstacleFilter::Instance().Obstacles().at(left_forward_obstacle_id)->Speed(),
                                           PlanningConfig::Instance().target_speed());
    maneuver_goal->target_s = ego_s + lookahead_distance;
    maneuver_goal->lane_id =
        incoming_way_point.lane_id < 0 ? incoming_way_point.lane_id + 1 : incoming_way_point.lane_id - 1;

  } else if (!can_change_left) {
    // only can change right
    const double lookahead_distance = std::max(PlanningConfig::Instance().min_lookahead_distance(),
                                               std::min(PlanningConfig::Instance().max_lookahead_distance(),
                                                        right_forward_clear_distance
                                                            - PlanningConfig::Instance().lon_safety_buffer()));
    maneuver_goal->decision_type = DecisionType::kChangeRight;
    maneuver_goal->target_speed =
        std::min(ObstacleFilter::Instance().Obstacles().at(right_forward_obstacle_id)->Speed(),
                 PlanningConfig::Instance().target_speed());
    maneuver_goal->target_s = ego_s + lookahead_distance;
    maneuver_goal->lane_id =
        incoming_way_point.lane_id < 0 ? incoming_way_point.lane_id - 1 : incoming_way_point.lane_id + 1;

  } else {
    // can change both
    int target_lane_id = this->SelectLane();

  }
}

int FollowLaneState::SelectLane() const {

}

void FollowLaneState::GetLaneClearDistance(int lane_offset,
                                           double *forward_clear_distance,
                                           double *backward_clear_distance,
                                           int *forward_obstacle_id,
                                           int *backward_obstacle_id) const {
  double maneuver_forward_distance = PlanningConfig::Instance().max_lookahead_distance();
  double maneuver_backward_distance = PlanningConfig::Instance().max_lookback_distance();
  int front_obstacle_id = -1;
  int rear_obstacle_id = -1;
  SLBoundary ego_sl_boundary;
  reference_line_->GetSLBoundary(VehicleState::Instance().GetEgoBox(), &ego_sl_boundary);
  double forward_clear_dist = maneuver_forward_distance;
  double backward_clear_dist = maneuver_backward_distance;
  const double ego_width = PlanningConfig::Instance().vehicle_params().width;
  for (const auto &obstacle : ObstacleFilter::Instance().Obstacles()) {
    SLBoundary obstacle_sl_boundary;
    reference_line_->GetSLBoundary(obstacle.second->BoundingBox(), &obstacle_sl_boundary);
    if (obstacle_sl_boundary.start_s - ego_sl_boundary.end_s > maneuver_forward_distance) {
      continue;
    }
    if (obstacle_sl_boundary.end_s + maneuver_backward_distance < ego_sl_boundary.start_s) {
      continue;
    }
    double left_lane_width, right_lane_width;
    double s = (obstacle_sl_boundary.start_s + obstacle_sl_boundary.end_s) * 0.5;
    reference_line_->GetLaneWidth(s, &left_lane_width, &right_lane_width);
    switch (lane_offset) {
      case 0: {
        if (reference_line_->IsOnLane(obstacle_sl_boundary)) {
          double driving_width = reference_line_->GetDrivingWidth(obstacle_sl_boundary);
          if (driving_width < ego_width + PlanningConfig::Instance().lat_safety_buffer()) {
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
        // the obstacle is on right lane, and the driving width on right lane is not safe
        if (obstacle_sl_boundary.end_l < -right_lane_width && std::fabs(obstacle_sl_boundary.end_l) <
            right_lane_width + ego_width + PlanningConfig::Instance().lat_safety_buffer()) {
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
        // the obstacle is on left lane, and the driving width on left lane is not safe
        if (obstacle_sl_boundary.start_l > left_lane_width && obstacle_sl_boundary.start_l <
            left_lane_width + ego_width + PlanningConfig::Instance().lat_safety_buffer()) {
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
