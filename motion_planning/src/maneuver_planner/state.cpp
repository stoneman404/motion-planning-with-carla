#include "traffic_lights/traffic_light_list.hpp"
#include "obstacle_filter/obstacle_filter.hpp"
#include "maneuver_planner/state.hpp"

namespace planning {

void State::GetLaneClearDistance(int lane_offset,
                                 const SLBoundary &ego_sl_boundary,
                                 const std::shared_ptr<ReferenceLine> &reference_line,
                                 double *forward_clear_distance,
                                 double *backward_clear_distance,
                                 int *forward_obstacle_id,
                                 int *backward_obstacle_id) const {
  double maneuver_forward_distance = PlanningConfig::Instance().max_lookahead_distance();
  double maneuver_backward_distance = PlanningConfig::Instance().max_lookback_distance();
  int front_obstacle_id = -1;
  int rear_obstacle_id = -1;
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

void State::TrafficLightDecision(std::shared_ptr<ReferenceLine> &reference_line,
                                 const SLPoint &ego_sl_point,
                                 ManeuverGoal *maneuver_goal) const {
  size_t nearest_index;

  const auto traffic_lights = TrafficLightList::Instance().TrafficLights();
  int min_id = -1;

  double min_dist = std::numeric_limits<double>::max();
  SLPoint nearest_light_sl_point;
  for (const auto &traffic_light : traffic_lights) {
    auto traffic_light_center = traffic_light.second->GetBox2d().Center();
    SLPoint sl_point;
    reference_line->XYToSL(traffic_light_center, &sl_point);
    if (sl_point.s < ego_sl_point.s || !reference_line->IsOnLane(sl_point)) {
      continue;
    }
    double dist = sl_point.s - ego_sl_point.s;
    if (dist > PlanningConfig::Instance().max_lookahead_distance()) {
      continue;
    }
    if (dist < min_dist) {
      min_dist = dist;
      min_id = traffic_light.first;
      nearest_light_sl_point = sl_point;
    }
  }
  const auto traffic_state = traffic_lights.at(min_id)->TrafficLightStatus().state;
  double ego_s = ego_sl_point.s;
  auto nearest_waypoint = reference_line->NearestWayPoint(
      TrafficLightList::Instance().TrafficLights().at(min_id)->GetBox2d().center_x(),
      TrafficLightList::Instance().TrafficLights().at(min_id)->GetBox2d().center_y(),
      &nearest_index);
  if (traffic_state == carla_msgs::CarlaTrafficLightStatus::GREEN
      || traffic_state == carla_msgs::CarlaTrafficLightStatus::OFF
      || traffic_state == carla_msgs::CarlaTrafficLightStatus::UNKNOWN) {
    maneuver_goal->maneuver_infos.resize(1);
    maneuver_goal->maneuver_infos.front().maneuver_target.target_speed = PlanningConfig::Instance().target_speed();
    maneuver_goal->maneuver_infos.front().has_stop_point = false;
    maneuver_goal->maneuver_infos.front().lane_id = nearest_waypoint.lane_id;
    maneuver_goal->maneuver_infos.front().ptr_ref_line = reference_line;
    maneuver_goal->decision_type = DecisionType::kFollowLane;
  } else {
    maneuver_goal->maneuver_infos.resize(1);
    maneuver_goal->maneuver_infos.front().has_stop_point = true;
    maneuver_goal->maneuver_infos.front().maneuver_target.target_s =
        std::min(nearest_light_sl_point.s,
                 ego_s + PlanningConfig::Instance().max_lookahead_distance());
    maneuver_goal->maneuver_infos.front().lane_id = nearest_waypoint.lane_id;
    maneuver_goal->maneuver_infos.front().ptr_ref_line = reference_line;
    maneuver_goal->decision_type = DecisionType::kStopAtTrafficSign;
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