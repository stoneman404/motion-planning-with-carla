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
}