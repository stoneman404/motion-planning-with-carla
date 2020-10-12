#include "traffic_lights/traffic_light_list.hpp"
#include "obstacle_filter/obstacle_filter.hpp"
#include "maneuver_planner/state.hpp"

namespace planning {

void State::GetLaneClearDistance(int lane_offset,
                                 std::shared_ptr<ReferenceLine> reference_line,
                                 double *forward_clear_distance,
                                 double *backward_clear_distance,
                                 int *forward_obstacle_id,
                                 int *backward_obstacle_id) const {
  double maneuver_forward_distance = PlanningConfig::Instance().max_lookahead_distance();
  double maneuver_backward_distance = PlanningConfig::Instance().max_lookback_distance();
  int front_obstacle_id = -1;
  int rear_obstacle_id = -1;
  SLBoundary ego_sl_boundary;
  reference_line->GetSLBoundary(VehicleState::Instance().GetEgoBox(), &ego_sl_boundary);
  double forward_clear_dist = maneuver_forward_distance;
  double backward_clear_dist = maneuver_backward_distance;
  const double ego_width = PlanningConfig::Instance().vehicle_params().width;
  for (const auto &obstacle : ObstacleFilter::Instance().Obstacles()) {
    SLBoundary obstacle_sl_boundary;
    reference_line->GetSLBoundary(obstacle.second->BoundingBox(), &obstacle_sl_boundary);
    if (obstacle_sl_boundary.start_s - ego_sl_boundary.end_s > maneuver_forward_distance) {
      continue;
    }
    if (obstacle_sl_boundary.end_s + maneuver_backward_distance < ego_sl_boundary.start_s) {
      continue;
    }
    double left_lane_width, right_lane_width;
    double s = (obstacle_sl_boundary.start_s + obstacle_sl_boundary.end_s) * 0.5;
    reference_line->GetLaneWidth(s, &left_lane_width, &right_lane_width);
    switch (lane_offset) {
      case 0: {
        if (reference_line->IsOnLane(obstacle_sl_boundary)) {
          double driving_width = reference_line->GetDrivingWidth(obstacle_sl_boundary);
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
            backward_clear_dist = std::fabs(backward_dist);
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
            backward_clear_dist = std::fabs(backward_dist);
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

void State::TrafficLightDecision(std::shared_ptr<ReferenceLine> reference_line,
                                 ManeuverGoal *maneuver_goal) const {
  size_t nearest_index;

  bool prohibited = false;
  const double max_decel = PlanningConfig::Instance().max_acc();
  const double min_stop_distance = std::pow(VehicleState::Instance().linear_vel(), 2) / (2.0 * max_decel);
  const double max_comfort_decel = PlanningConfig::Instance().max_acc() * 0.6;
  double min_comfort_stop_distance = std::pow(VehicleState::Instance().linear_vel(), 2) / (2.0 * max_comfort_decel);

  const auto traffic_lights = TrafficLightList::Instance().TrafficLights();
  const auto ego_box = VehicleState::Instance().GetEgoBox();
  SLBoundary ego_sl_boundary;
  reference_line->GetSLBoundary(ego_box, &ego_sl_boundary);
  int min_id = -1;
  double min_dist = std::numeric_limits<double>::max();
  SLBoundary nearest_traffic_light_sl_boundary;
  for (const auto &traffic_light : traffic_lights) {
    SLBoundary sl_boundary;
    const Box2d light_box = traffic_light.second->GetBox2d();
    reference_line->GetSLBoundary(light_box, &sl_boundary);
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
  auto nearest_waypoint = reference_line->NearestWayPoint(
      TrafficLightList::Instance().TrafficLights().at(min_id)->GetBox2d().center_x(),
      TrafficLightList::Instance().TrafficLights().at(min_id)->GetBox2d().center_y(),
      &nearest_index);
  if (traffic_state == carla_msgs::CarlaTrafficLightStatus::GREEN
      || traffic_state == carla_msgs::CarlaTrafficLightStatus::OFF
      || traffic_state == carla_msgs::CarlaTrafficLightStatus::UNKNOWN) {
    prohibited = false;
  } else if (min_dist < min_stop_distance) {
    prohibited = true;
    maneuver_goal->maneuver_infos.resize(1);
    maneuver_goal->maneuver_infos.front().has_stop_point = true;
    maneuver_goal->decision_type = DecisionType::kEmergencyStop;
    maneuver_goal->maneuver_infos.front().lane_id = nearest_waypoint.lane_id;
    maneuver_goal->maneuver_infos.front().ptr_ref_line = reference_line;
    maneuver_goal->maneuver_infos.front().maneuver_target.target_s =
        std::max(nearest_traffic_light_sl_boundary.start_s, ego_s + min_stop_distance);

  } else if (min_dist > min_comfort_stop_distance) {
    prohibited = false;
  } else {
    prohibited = true;
    maneuver_goal->maneuver_infos.resize(1);
    maneuver_goal->maneuver_infos.front().has_stop_point = true;
    maneuver_goal->maneuver_infos.front().maneuver_target.target_s =
        std::max(nearest_traffic_light_sl_boundary.start_s, ego_s + min_stop_distance);
    maneuver_goal->maneuver_infos.front().lane_id = nearest_waypoint.lane_id;
    maneuver_goal->maneuver_infos.front().ptr_ref_line = reference_line;
    maneuver_goal->decision_type = DecisionType::kStopAtTrafficSign;
  }
  if (!prohibited) {
    maneuver_goal->maneuver_infos.resize(1);
    maneuver_goal->maneuver_infos.front().maneuver_target.target_speed = PlanningConfig::Instance().target_speed();
    maneuver_goal->maneuver_infos.front().has_stop_point = false;
    maneuver_goal->maneuver_infos.front().lane_id = nearest_waypoint.lane_id;
    maneuver_goal->maneuver_infos.front().ptr_ref_line = reference_line;
    maneuver_goal->decision_type = DecisionType::kFollowLane;
  }
}

ManeuverGoal State::CombineManeuver(const ManeuverGoal &traffic_light_maneuver,
                                    const ManeuverGoal &obstacle_maneuver) const {
  if (traffic_light_maneuver.decision_type == DecisionType::kEmergencyStop) {
    return traffic_light_maneuver;
  }

  if (obstacle_maneuver.decision_type == DecisionType::kEmergencyStop) {
    return obstacle_maneuver;
  }
  ManeuverGoal combined_maneuver;
  if (traffic_light_maneuver.decision_type == DecisionType::kStopAtTrafficSign &&
      obstacle_maneuver.decision_type == DecisionType::kStopAtDestination) {
    combined_maneuver.maneuver_infos.resize(1);
    combined_maneuver.maneuver_infos.front().has_stop_point = true;
//    combined_maneuver.target_speed = 0.0;
    combined_maneuver.maneuver_infos.front().lane_id = traffic_light_maneuver.maneuver_infos.front().lane_id;
    combined_maneuver.decision_type = traffic_light_maneuver.maneuver_infos.front().maneuver_target.target_s <
        obstacle_maneuver.maneuver_infos.front().maneuver_target.target_s ?
                                      DecisionType::kStopAtTrafficSign : DecisionType::kStopAtDestination;
    combined_maneuver.maneuver_infos.front().maneuver_target.target_s =
        std::min(traffic_light_maneuver.maneuver_infos.front().maneuver_target.target_s,
                 obstacle_maneuver.maneuver_infos.front().maneuver_target.target_s);
    combined_maneuver.maneuver_infos.front().ptr_ref_line =
        traffic_light_maneuver.maneuver_infos.front().maneuver_target.target_s <
            obstacle_maneuver.maneuver_infos.front().maneuver_target.target_s
        ? traffic_light_maneuver.maneuver_infos.front().ptr_ref_line :
        obstacle_maneuver.maneuver_infos.front().ptr_ref_line;
    return combined_maneuver;
  }
  return obstacle_maneuver;
}

}