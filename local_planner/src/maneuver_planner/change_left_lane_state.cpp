#include "obstacle_filter/obstacle_filter.hpp"
#include "maneuver_planner/change_left_lane_state.hpp"
namespace planning {

bool ChangeLeftLaneState::Enter(ManeuverPlanner *maneuver_planner) {
  // todo: set offset for ego_pose
  const auto ego_pose = VehicleState::Instance().pose();
  const auto goal_pose = PlanningContext::Instance().global_goal_pose().pose;
  if (maneuver_planner->NeedReRoute()) {
    planning_srvs::RouteResponse route_response;
    bool result = maneuver_planner->ReRoute(ego_pose, goal_pose, route_response);
    if (!result) {
      ROS_FATAL("Failed to enter **ChangeLeftLaneState** state");
      return false;
    }
    auto &route_infos = PlanningContext::Instance().mutable_route_infos();
    if (route_infos.size() > 2) {
      route_infos.pop_front();
    }
    route_infos.push_back(route_response);
    std::list<std::shared_ptr<ReferenceLine>> reference_lines;
    bool reference_result = ManeuverPlanner::UpdateReferenceLine(route_infos, &reference_lines);
    if (!reference_result) {
      ROS_FATAL("Failed to enter **ChangeLeftLaneState**, because UpdateReferenceLine failed");
      return false;
    }
    auto &reference_line_list = PlanningContext::Instance().mutable_reference_lines();
    reference_line_list = reference_lines;
  }

  target_lane_id_ = maneuver_planner->maneuver_goal().lane_id;
  current_lane_id_ = VehicleState::Instance().lane_id();
  ROS_ASSERT(PlanningContext::Instance().reference_lines().size() == 2);
  current_reference_line_ = PlanningContext::Instance().reference_lines().front();
  target_reference_line_ = PlanningContext::Instance().reference_lines().back();
}

bool ChangeLeftLaneState::Execute(ManeuverPlanner *maneuver_planner) {

  if (maneuver_planner == nullptr) {
    ROS_FATAL("[ChangeLeftLaneState::Execute] failed, maneuver_planner == nullptr");
    return false;
  }
  // todo: trajectory motion_planner

  return false;
}
void ChangeLeftLaneState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are currently exit the **ChangeLeftLaneState**");
}

State &ChangeLeftLaneState::Instance() {
  static ChangeLeftLaneState instance;
  return instance;
}

std::string ChangeLeftLaneState::Name() const { return "ChangeLeftLaneState"; }

State *ChangeLeftLaneState::NextState(ManeuverPlanner *maneuver_planner) const {
  if (maneuver_planner == nullptr) {
    return nullptr;
  }
  ManeuverGoal obstacle_maneuver;
  this->ObstacleDecision(&obstacle_maneuver);
  ManeuverGoal traffic_light_maneuver;
  this->TrafficLightDecision(target_reference_line_, &traffic_light_maneuver);
  maneuver_planner->SetManeuverGoal(obstacle_maneuver);
  switch (obstacle_maneuver.decision_type) {
    case DecisionType::kChangeLeft:return &(ChangeLeftLaneState::Instance());
    case DecisionType::kFollowLane:return &(FollowLaneState::Instance());
    case DecisionType::kEmergencyStop:return &(EmergencyStopState::Instance());
    case DecisionType::kStopAtDestination:
    case DecisionType::kChangeRight:
    case DecisionType::kStopAtTrafficSign:
    default:return &(StopState::Instance());

  }
}


void ChangeLeftLaneState::ObstacleDecision(ManeuverGoal *maneuver_goal) const {
  double leading_clear_distance;
  double following_clear_distance;
  int leading_vehicle_id;
  int following_vehicle_id;
  if (current_lane_id_ == target_lane_id_) {
    // enter the target_lane
    SLPoint ego_sl;
    target_reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                                   VehicleState::Instance().pose().position.y, &ego_sl);

    this->GetLaneClearDistance(0, target_reference_line_,
                               &leading_clear_distance,
                               &following_clear_distance,
                               &leading_vehicle_id,
                               &following_vehicle_id);
    if (ego_sl.s + leading_clear_distance < target_reference_line_->Length()) {
      if (leading_vehicle_id < 0 && following_vehicle_id < 0) {
        // has no leading and following vehicle
        maneuver_goal->decision_type = DecisionType::kFollowLane;
        maneuver_goal->target_s =
            ego_sl.s + std::min(std::max(
                leading_clear_distance, PlanningConfig::Instance().min_lookahead_distance()),
                                PlanningConfig::Instance().max_lookahead_distance());
        maneuver_goal->target_speed = PlanningConfig::Instance().target_speed();
        maneuver_goal->lane_id = target_lane_id_;
        maneuver_goal->has_stop_point = false;
      } else if (leading_vehicle_id > 0 && following_vehicle_id < 0) {
        // has leading vehicle but no following vehicle
        if (leading_clear_distance > PlanningConfig::Instance().lon_safety_buffer()) {
          // keep lane
          maneuver_goal->decision_type = DecisionType::kFollowLane;
          maneuver_goal->target_s =
              ego_sl.s + std::min(std::max(
                  leading_clear_distance, PlanningConfig::Instance().min_lookahead_distance()),
                                  PlanningConfig::Instance().max_lookahead_distance());
          maneuver_goal->target_speed = std::min(
              PlanningConfig::Instance().target_speed(),
              ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());
          maneuver_goal->has_stop_point = false;
        } else {
          // emergency stop
          maneuver_goal->decision_type = DecisionType::kEmergencyStop;
          maneuver_goal->target_s =
              ego_sl.s + std::min(std::max(
                  leading_clear_distance, PlanningConfig::Instance().min_lookahead_distance()),
                                  PlanningConfig::Instance().max_lookahead_distance());
          maneuver_goal->target_speed = 0.0;
          maneuver_goal->has_stop_point = true;
          maneuver_goal->lane_id = target_lane_id_;
        }
      } else if (leading_vehicle_id > 0 && following_vehicle_id > 0) {
        // has leading and following vehicle
        if (leading_clear_distance < PlanningConfig::Instance().lon_safety_buffer() ||
            following_clear_distance < PlanningConfig::Instance().lon_safety_buffer()) {
          maneuver_goal->has_stop_point = true;
          maneuver_goal->target_s = ego_sl.s + PlanningConfig::Instance().lon_safety_buffer();
          maneuver_goal->target_speed = 0.0;
          maneuver_goal->decision_type = DecisionType::kEmergencyStop;
          maneuver_goal->lane_id = target_lane_id_;
        } else {
          maneuver_goal->decision_type = DecisionType::kFollowLane;
          maneuver_goal->target_s =
              ego_sl.s + std::min(std::max(
                  leading_clear_distance - PlanningConfig::Instance().lon_safety_buffer(),
                  PlanningConfig::Instance().min_lookahead_distance()),
                                  PlanningConfig::Instance().max_lookahead_distance());
          maneuver_goal->target_speed = std::min(
              PlanningConfig::Instance().target_speed(),
              ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());
          maneuver_goal->lane_id = target_lane_id_;
          maneuver_goal->has_stop_point = false;
        }
      } else {
        // has following vehicle but no leading vehicle
        maneuver_goal->decision_type = DecisionType::kFollowLane;
        maneuver_goal->target_s =
            ego_sl.s + std::min(std::max(leading_clear_distance,
                                         PlanningConfig::Instance().min_lookahead_distance()),
                                PlanningConfig::Instance().max_lookahead_distance());
        maneuver_goal->target_speed = std::max(
            PlanningConfig::Instance().target_speed(),
            ObstacleFilter::Instance().Obstacles().at(following_vehicle_id)->Speed());
        maneuver_goal->lane_id = target_lane_id_;
        maneuver_goal->has_stop_point = false;
      }
    } else {
      // close to reference line's end
      maneuver_goal->lane_id = target_lane_id_;
      maneuver_goal->target_s = target_reference_line_->Length();
      maneuver_goal->target_speed = 0.0;
      maneuver_goal->decision_type = DecisionType::kStopAtDestination;
      maneuver_goal->has_stop_point = true;
    }
  } else {
    // not in target lane
    // 1.current lane
    SLPoint current_ego_sl;
    current_reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                                    VehicleState::Instance().pose().position.y,
                                    &current_ego_sl);

    this->GetLaneClearDistance(0, current_reference_line_,
                               &leading_clear_distance,
                               &following_clear_distance,
                               &leading_vehicle_id,
                               &following_vehicle_id);
    if (leading_clear_distance < PlanningConfig::Instance().lon_safety_buffer()) {
      maneuver_goal->target_s = leading_clear_distance;
      maneuver_goal->decision_type = DecisionType::kEmergencyStop;
      maneuver_goal->has_stop_point = true;
      maneuver_goal->lane_id = current_lane_id_;
      maneuver_goal->target_speed = 0.0;
    } else if (current_ego_sl.s + leading_clear_distance > current_reference_line_->Length()) {
      // near the current reference line's end
      maneuver_goal->has_stop_point = true;
      maneuver_goal->target_speed = 0.0;
      maneuver_goal->decision_type = DecisionType::kStopAtDestination;
      maneuver_goal->lane_id = current_lane_id_;
      maneuver_goal->target_s = current_reference_line_->Length();
    } else {
      // not the current reference line's end, continue change lane or return to the current lane
      if (leading_clear_distance < PlanningConfig::Instance().maneuver_forward_clear_threshold()
          || following_clear_distance < PlanningConfig::Instance().maneuver_backward_clear_threshold()) {
        // not safe to continue change lane, return to current lane
        maneuver_goal->lane_id = current_lane_id_;
        maneuver_goal->target_speed = leading_vehicle_id < 0 ? PlanningConfig::Instance().target_speed() :
                                      std::min(PlanningConfig::Instance().target_speed(),
                                               ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());
        maneuver_goal->has_stop_point = false;
        maneuver_goal->target_s = std::min(PlanningConfig::Instance().max_lookahead_distance(),
                                           std::max(leading_clear_distance,
                                                    PlanningConfig::Instance().min_lookahead_distance()));
        maneuver_goal->decision_type = DecisionType::kFollowLane;
      } else {
        // can continue change lane, but should check the target lane's condition
        double target_leading_clear_distance, target_following_clear_distance;
        int target_leading_vehicle_id, target_following_vehicle_id;
        this->GetLaneClearDistance(0, target_reference_line_,
                                   &target_leading_clear_distance,
                                   &target_following_clear_distance,
                                   &target_leading_vehicle_id,
                                   &target_following_vehicle_id);
        if (target_leading_clear_distance < PlanningConfig::Instance().maneuver_target_lane_forward_clear_threshold()
            || target_following_clear_distance
                < PlanningConfig::Instance().maneuver_target_lane_backward_clear_threshold()) {
          // cannot change lane, because the clear distance is too short
          maneuver_goal->lane_id = current_lane_id_;
          maneuver_goal->target_speed = leading_vehicle_id < 0 ? PlanningConfig::Instance().target_speed() :
                                        std::min(PlanningConfig::Instance().target_speed(),
                                                 ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());
          maneuver_goal->has_stop_point = false;
          maneuver_goal->target_s = current_ego_sl.s + std::min(PlanningConfig::Instance().max_lookahead_distance(),
                                                                std::max(leading_clear_distance,
                                                                         PlanningConfig::Instance().min_lookahead_distance()));
          maneuver_goal->decision_type = DecisionType::kFollowLane;
        } else {
          //  can continue change lane
          SLPoint ego_sl_target_lane;
          target_reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                                         VehicleState::Instance().pose().position.y,
                                         &ego_sl_target_lane);
          maneuver_goal->lane_id = target_lane_id_;
          maneuver_goal->target_s = ego_sl_target_lane.s + std::min(
              PlanningConfig::Instance().max_lookahead_distance(),
              std::max(target_leading_clear_distance,
                       PlanningConfig::Instance().min_lookahead_distance()));
          maneuver_goal->decision_type = DecisionType::kChangeLeft;
          maneuver_goal->target_speed = target_leading_vehicle_id < 0 ? PlanningConfig::Instance().target_speed() :
                                        std::min(PlanningConfig::Instance().target_speed(),
                                                 ObstacleFilter::Instance().Obstacles().at(target_leading_vehicle_id)->Speed());
          maneuver_goal->has_stop_point = false;
        }
      }
    }
  }
}

}