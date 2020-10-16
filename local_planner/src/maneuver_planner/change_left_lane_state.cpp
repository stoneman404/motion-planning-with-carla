#include <tf/transform_datatypes.h>
#include "obstacle_filter/obstacle_filter.hpp"
#include "maneuver_planner/change_left_lane_state.hpp"
namespace planning {

bool ChangeLeftLaneState::Enter(ManeuverPlanner *maneuver_planner) {
  if (maneuver_planner->NeedReRoute()) {

  }
  auto &maneuver_goal = maneuver_planner->multable_maneuver_goal();
  if (maneuver_goal.maneuver_infos.size() < 2) {
    ROS_DEBUG("Failed to enter **ChangeLeftLaneState** state, because the maneuver goal is less than 2");
  }
  for (auto &maneuver_info : maneuver_goal.maneuver_infos) {
    if (maneuver_info.ptr_ref_line == nullptr) {
      const auto ego_pose = VehicleState::Instance().pose();
      const double theta = MathUtil::NormalizeAngle(tf::getYaw(ego_pose.orientation));
      const double sin_theta = std::sin(theta);
      const double cos_theta = std::cos(theta);
      const double offset_length = VehicleState::Instance().ego_waypoint().lane_width;
      auto new_pose = ego_pose;
      new_pose.position.x = ego_pose.position.x - sin_theta * offset_length;
      new_pose.position.y = ego_pose.position.y + cos_theta * offset_length;
      const auto goal_pose = PlanningContext::Instance().global_goal_pose().pose;
      planning_srvs::RouteResponse route_response;
      bool result = maneuver_planner->ReRoute(new_pose, goal_pose, route_response);
      if (!result) {
        ROS_FATAL("Failed to enter **ChangeLeftLaneState** state");
        return false;
      }
      PlanningContext::Instance().mutable_route_infos().push_back(route_response);
    }
  }

  after_lane_id_ = maneuver_goal.maneuver_infos.back().lane_id;
  before_lane_id_ = maneuver_goal.maneuver_infos.front().lane_id;

  before_reference_line_ = maneuver_goal.maneuver_infos.front().ptr_ref_line;
  after_reference_line_ = maneuver_goal.maneuver_infos.back().ptr_ref_line;
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
  this->TrafficLightDecision(after_reference_line_, &traffic_light_maneuver);
  maneuver_planner->SetManeuverGoal(obstacle_maneuver);
  switch (obstacle_maneuver.decision_type) {
    case DecisionType::kChangeLeft: return &(ChangeLeftLaneState::Instance());
    case DecisionType::kFollowLane: return &(FollowLaneState::Instance());
    case DecisionType::kEmergencyStop: return &(EmergencyStopState::Instance());
    case DecisionType::kStopAtDestination:
    case DecisionType::kChangeRight:
    case DecisionType::kStopAtTrafficSign:
    default: return &(StopState::Instance());

  }
}

void ChangeLeftLaneState::ObstacleDecision(ManeuverGoal *maneuver_goal) const {
  double leading_clear_distance;
  double following_clear_distance;
  int leading_vehicle_id;
  int following_vehicle_id;
  SLBoundary sl_boundary;
  after_reference_line_->GetSLBoundary(VehicleState::Instance().GetEgoBox(), &sl_boundary);
  if (after_reference_line_->IsOnLane(sl_boundary)) {
    // enter the target_lane
    SLPoint ego_sl;
    after_reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                                  VehicleState::Instance().pose().position.y, &ego_sl);

    this->GetLaneClearDistance(0, after_reference_line_,
                               &leading_clear_distance,
                               &following_clear_distance,
                               &leading_vehicle_id,
                               &following_vehicle_id);
    if (ego_sl.s + leading_clear_distance < after_reference_line_->Length()) {
      if (leading_vehicle_id < 0 && following_vehicle_id < 0) {
        // has no leading and following vehicle
        maneuver_goal->maneuver_infos.resize(1);
        maneuver_goal->decision_type = DecisionType::kFollowLane;
        maneuver_goal->maneuver_infos.front().maneuver_target.target_speed = PlanningConfig::Instance().target_speed();
        maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
        maneuver_goal->maneuver_infos.front().has_stop_point = false;
        maneuver_goal->maneuver_infos.front().ptr_ref_line = after_reference_line_;
      } else if (leading_vehicle_id > 0 && following_vehicle_id < 0) {
        // has leading vehicle but no following vehicle
        if (leading_clear_distance > PlanningConfig::Instance().lon_safety_buffer()) {
          // keep lane
          maneuver_goal->maneuver_infos.resize(1);
          maneuver_goal->decision_type = DecisionType::kFollowLane;
          maneuver_goal->maneuver_infos.front().maneuver_target.target_speed = std::min(
              PlanningConfig::Instance().target_speed(),
              ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());
          maneuver_goal->maneuver_infos.front().has_stop_point = false;
          maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
          maneuver_goal->maneuver_infos.front().ptr_ref_line = after_reference_line_;

        } else {
          // emergency stop
          maneuver_goal->decision_type = DecisionType::kEmergencyStop;
          maneuver_goal->maneuver_infos.resize(1);
          maneuver_goal->maneuver_infos.front().maneuver_target.target_s =
              ego_sl.s + std::min(std::max(leading_clear_distance,
                                           PlanningConfig::Instance().min_lookahead_distance()),
                                  PlanningConfig::Instance().max_lookahead_distance());
          maneuver_goal->maneuver_infos.front().has_stop_point = true;
          maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
          maneuver_goal->maneuver_infos.front().ptr_ref_line = after_reference_line_;
        }
      } else if (leading_vehicle_id > 0 && following_vehicle_id > 0) {
        // has leading and following vehicle
        if (leading_clear_distance < PlanningConfig::Instance().lon_safety_buffer() ||
            following_clear_distance < PlanningConfig::Instance().lon_safety_buffer()) {
          maneuver_goal->maneuver_infos.resize(1);
          maneuver_goal->maneuver_infos.front().has_stop_point = true;
          maneuver_goal->maneuver_infos.front().maneuver_target.target_s =
              ego_sl.s + PlanningConfig::Instance().lon_safety_buffer();
          maneuver_goal->decision_type = DecisionType::kEmergencyStop;
          maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
          maneuver_goal->maneuver_infos.front().ptr_ref_line = after_reference_line_;
        } else {
          maneuver_goal->decision_type = DecisionType::kFollowLane;
          maneuver_goal->maneuver_infos.resize(1);
          maneuver_goal->maneuver_infos.front().maneuver_target.target_speed = std::min(
              ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed(),
              PlanningConfig::Instance().target_speed());
          maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
          maneuver_goal->maneuver_infos.front().has_stop_point = false;
          maneuver_goal->maneuver_infos.front().ptr_ref_line = after_reference_line_;
        }
      } else {
        // has following vehicle but no leading vehicle
        maneuver_goal->decision_type = DecisionType::kFollowLane;
        maneuver_goal->maneuver_infos.resize(1);
        maneuver_goal->maneuver_infos.front().maneuver_target.target_speed =
            std::min(PlanningConfig::Instance().target_speed(),
                     ObstacleFilter::Instance().Obstacles().at(following_vehicle_id)->Speed());
        maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
        maneuver_goal->maneuver_infos.front().has_stop_point = false;
        maneuver_goal->maneuver_infos.front().ptr_ref_line = after_reference_line_;
      }
    } else {
      // close to reference line's end
      maneuver_goal->decision_type = DecisionType::kStopAtDestination;
      maneuver_goal->maneuver_infos.resize(1);
      maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
      maneuver_goal->maneuver_infos.front().maneuver_target.target_s = after_reference_line_->Length();
      maneuver_goal->maneuver_infos.front().has_stop_point = true;
      maneuver_goal->maneuver_infos.front().ptr_ref_line = after_reference_line_;
    }
  } else {
    // not in target lane
    // 1.current lane
    SLPoint current_ego_sl;
    before_reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                                   VehicleState::Instance().pose().position.y,
                                   &current_ego_sl);

    this->GetLaneClearDistance(0, before_reference_line_,
                               &leading_clear_distance,
                               &following_clear_distance,
                               &leading_vehicle_id,
                               &following_vehicle_id);
    if (leading_clear_distance < PlanningConfig::Instance().lon_safety_buffer()) {
      maneuver_goal->decision_type = DecisionType::kEmergencyStop;
      maneuver_goal->maneuver_infos.resize(1);
      maneuver_goal->maneuver_infos.front().maneuver_target.target_s = leading_clear_distance;
      maneuver_goal->maneuver_infos.front().has_stop_point = true;
      maneuver_goal->maneuver_infos.front().lane_id = before_lane_id_;
      maneuver_goal->maneuver_infos.front().ptr_ref_line = before_reference_line_;

    } else if (current_ego_sl.s + leading_clear_distance > before_reference_line_->Length()) {
      // near the current reference line's end
      maneuver_goal->decision_type = DecisionType::kStopAtDestination;
      maneuver_goal->maneuver_infos.resize(1);
      maneuver_goal->maneuver_infos.front().has_stop_point = true;
      maneuver_goal->maneuver_infos.front().lane_id = before_lane_id_;
      maneuver_goal->maneuver_infos.front().maneuver_target.target_s = before_reference_line_->Length();
      maneuver_goal->maneuver_infos.front().ptr_ref_line = before_reference_line_;
    } else {
      // not the current reference line's end, continue change lane or return to the current lane
      if (leading_clear_distance < PlanningConfig::Instance().maneuver_forward_clear_threshold()
          || following_clear_distance < PlanningConfig::Instance().maneuver_backward_clear_threshold()) {
        // not safe to continue change lane, return to current lane
        maneuver_goal->decision_type = DecisionType::kFollowLane;
        maneuver_goal->maneuver_infos.resize(1);
        maneuver_goal->maneuver_infos.front().lane_id = before_lane_id_;
        maneuver_goal->maneuver_infos.front().maneuver_target.target_speed =
            leading_vehicle_id < 0 ? PlanningConfig::Instance().target_speed() :
            std::min(PlanningConfig::Instance().target_speed(),
                     ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());
        maneuver_goal->maneuver_infos.front().has_stop_point = false;
        maneuver_goal->maneuver_infos.front().ptr_ref_line = before_reference_line_;
      } else {
        // can continue change lane, but should check the target lane's condition
        double target_leading_clear_distance, target_following_clear_distance;
        int target_leading_vehicle_id, target_following_vehicle_id;
        this->GetLaneClearDistance(0, after_reference_line_,
                                   &target_leading_clear_distance,
                                   &target_following_clear_distance,
                                   &target_leading_vehicle_id,
                                   &target_following_vehicle_id);
        if (target_leading_clear_distance < PlanningConfig::Instance().maneuver_target_lane_forward_clear_threshold()
            || target_following_clear_distance
                < PlanningConfig::Instance().maneuver_target_lane_backward_clear_threshold()) {
          // cannot change lane, because the clear distance is too short
          maneuver_goal->maneuver_infos.resize(1);
          maneuver_goal->maneuver_infos.front().lane_id = before_lane_id_;
          maneuver_goal->maneuver_infos.front().maneuver_target.target_speed =
              leading_vehicle_id < 0 ? PlanningConfig::Instance().target_speed() :
              std::min(PlanningConfig::Instance().target_speed(),
                       ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());
          maneuver_goal->maneuver_infos.front().has_stop_point = false;
          maneuver_goal->maneuver_infos.front().ptr_ref_line = before_reference_line_;
          maneuver_goal->decision_type = DecisionType::kFollowLane;
        } else {
          //  can continue change lane
          SLPoint ego_sl_target_lane;
          after_reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                                        VehicleState::Instance().pose().position.y,
                                        &ego_sl_target_lane);
          // 1. current lane maneuver info

          maneuver_goal->decision_type = DecisionType::kChangeLeft;
          maneuver_goal->maneuver_infos.resize(2);
          maneuver_goal->maneuver_infos.front().maneuver_target.target_speed =
              leading_vehicle_id < 0 ? PlanningConfig::Instance().target_speed() :
              std::min(PlanningConfig::Instance().target_speed(),
                       ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());
          maneuver_goal->maneuver_infos.front().ptr_ref_line = before_reference_line_;
          maneuver_goal->maneuver_infos.front().lane_id = before_lane_id_;
          maneuver_goal->maneuver_infos.front().has_stop_point = false;

          // 2. target lane maneuver info
          maneuver_goal->maneuver_infos.back().lane_id = after_lane_id_;
          maneuver_goal->maneuver_infos.back().maneuver_target.target_speed =
              target_leading_vehicle_id < 0 ? PlanningConfig::Instance().target_speed() :
              std::min(PlanningConfig::Instance().target_speed(),
                       ObstacleFilter::Instance().Obstacles().at(target_leading_vehicle_id)->Speed());
          maneuver_goal->maneuver_infos.back().has_stop_point = false;
          maneuver_goal->maneuver_infos.back().ptr_ref_line = after_reference_line_;
        }
      }
    }
  }
}

}