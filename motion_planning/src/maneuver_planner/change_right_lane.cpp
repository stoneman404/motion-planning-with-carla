#include <tf/transform_datatypes.h>
#include "obstacle_filter/obstacle_filter.hpp"
#include "maneuver_planner/follow_lane.hpp"
#include "maneuver_planner/emergency_stop.hpp"
#include "maneuver_planner/state.hpp"
#include "maneuver_planner/change_right_lane.hpp"

namespace planning {
bool ChangeRightLane::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current enter the **ChangeRightLane**");

  const auto ego_pose = VehicleState::Instance().ego_waypoint();
  const double theta = MathUtils::NormalizeAngle(tf::getYaw(ego_pose.pose.orientation));
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double offset_length = VehicleState::Instance().ego_waypoint().lane_width;
  auto new_pose = ego_pose.pose;
  new_pose.position.x = ego_pose.pose.position.x + sin_theta * offset_length;
  new_pose.position.y = ego_pose.pose.position.y + cos_theta * offset_length;
  const auto goal_pose = PlanningContext::Instance().global_goal_pose().pose;
  planning_srvs::RouteResponse route_response;
  bool result = maneuver_planner->ReRoute(new_pose, goal_pose, route_response);
  if (!result) {
    ROS_FATAL("Failed to enter **ChangeLeftLane** state");
    return false;
  }
  maneuver_planner->multable_routes().push_back(route_response);
  ROS_ASSERT(maneuver_planner->multable_routes().size() == 2);
}



void ChangeRightLane::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current exit the **ChangeRightLane**");
  switch (maneuver_planner->multable_maneuver_goal().decision_type) {
    case DecisionType::kFollowLane:
    case DecisionType::kEmergencyStop:
    case DecisionType::kStopAtTrafficSign:
    case DecisionType::kStopAtDestination: {
      if (maneuver_planner->multable_maneuver_goal().maneuver_infos.front().lane_id == after_lane_id_) {
        maneuver_planner->multable_routes().pop_front();
      } else {
        maneuver_planner->multable_routes().pop_back();
      }
      break;
    }
    case DecisionType::kChangeLeft:
    case DecisionType::kChangeRight:
    default:break;
  }
}

State &ChangeRightLane::Instance() {
  static ChangeRightLane instance;
  return instance;
}

std::string ChangeRightLane::Name() const { return "ChangeRightLane"; }

State *ChangeRightLane::Transition(ManeuverPlanner *maneuver_planner) {
  if (maneuver_planner == nullptr) {
    return nullptr;
  }
  before_reference_line_ = maneuver_planner->multable_ref_line().front();
  after_reference_line_ = maneuver_planner->multable_ref_line().back();
  SLPoint ego_sl;
  before_reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                                 VehicleState::Instance().pose().position.y,
                                 &ego_sl);
  before_lane_id_ = before_reference_line_->NearestWayPoint(ego_sl.s + 5.0).lane_id;
  after_reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                                VehicleState::Instance().pose().position.y,
                                &ego_sl);
  after_lane_id_ = after_reference_line_->NearestWayPoint(ego_sl.s + 5.0).lane_id;
  ManeuverGoal obstacle_maneuver;
  this->ObstacleDecision(&obstacle_maneuver);
  maneuver_planner->SetManeuverGoal(obstacle_maneuver);
  switch (obstacle_maneuver.decision_type) {
    case DecisionType::kChangeRight:return &(ChangeRightLane::Instance());
    case DecisionType::kFollowLane:return &(FollowLane::Instance());
    case DecisionType::kEmergencyStop:return &(EmergencyStop::Instance());
    case DecisionType::kStopAtDestination:
    case DecisionType::kChangeLeft:
    case DecisionType::kStopAtTrafficSign:
    default:return &(Stop::Instance());
  }
}

void ChangeRightLane::ObstacleDecision(ManeuverGoal *maneuver_goal) const {
  double leading_clear_distance;
  double following_clear_distance;
  int leading_vehicle_id;
  int following_vehicle_id;
  SLPoint ego_sl;
  after_reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                                VehicleState::Instance().pose().position.y, &ego_sl);
  if (IsOnLane(ego_sl.s, ego_sl.l, after_reference_line_)) {
    // enter the target_lane
    this->GetLaneClearDistance(0, after_reference_line_,
                               &leading_clear_distance,
                               &following_clear_distance,
                               &leading_vehicle_id,
                               &following_vehicle_id);
    if (ego_sl.s + leading_clear_distance < after_reference_line_->Length()) {
      if (leading_vehicle_id < 0 && following_vehicle_id < 0) {
        // has no leading and following vehicle
        maneuver_goal->decision_type = DecisionType::kFollowLane;
        maneuver_goal->maneuver_infos.resize(1);
        maneuver_goal->maneuver_infos.front().maneuver_target.target_speed = PlanningConfig::Instance().target_speed();
        maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
        maneuver_goal->maneuver_infos.front().ptr_ref_line = after_reference_line_;
        maneuver_goal->maneuver_infos.front().has_stop_point = false;
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
          maneuver_goal->maneuver_infos.resize(1);
          maneuver_goal->decision_type = DecisionType::kEmergencyStop;
          maneuver_goal->maneuver_infos.front().maneuver_target.target_s =
              ego_sl.s + std::min(std::max(
                  leading_clear_distance, PlanningConfig::Instance().min_lookahead_distance()),
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
              PlanningConfig::Instance().target_speed(),
              ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());
          maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
          maneuver_goal->maneuver_infos.front().has_stop_point = false;
          maneuver_goal->maneuver_infos.front().ptr_ref_line = after_reference_line_;
        }
      } else {
        // has following vehicle but no leading vehicle
        maneuver_goal->decision_type = DecisionType::kFollowLane;
        maneuver_goal->maneuver_infos.resize(1);
        maneuver_goal->maneuver_infos.front().maneuver_target.target_speed = std::max(
            PlanningConfig::Instance().target_speed(),
            ObstacleFilter::Instance().Obstacles().at(following_vehicle_id)->Speed());
        maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
        maneuver_goal->maneuver_infos.front().has_stop_point = false;
        maneuver_goal->maneuver_infos.front().ptr_ref_line = after_reference_line_;
      }
    } else {
      // close to reference line's end
      maneuver_goal->maneuver_infos.resize(1);
      maneuver_goal->maneuver_infos.front().lane_id = after_lane_id_;
      maneuver_goal->maneuver_infos.front().maneuver_target.target_s = after_reference_line_->Length();
      maneuver_goal->decision_type = DecisionType::kStopAtDestination;
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
      maneuver_goal->maneuver_infos.resize(1);
      maneuver_goal->maneuver_infos.front().maneuver_target.target_s = leading_clear_distance;
      maneuver_goal->decision_type = DecisionType::kEmergencyStop;
      maneuver_goal->maneuver_infos.front().has_stop_point = true;
      maneuver_goal->maneuver_infos.front().lane_id = before_lane_id_;
      maneuver_goal->maneuver_infos.front().ptr_ref_line = before_reference_line_;
    } else if (current_ego_sl.s + leading_clear_distance > before_reference_line_->Length()) {
      // near the current reference line's end
      maneuver_goal->maneuver_infos.resize(1);
      maneuver_goal->decision_type = DecisionType::kStopAtDestination;
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
        if (target_leading_clear_distance <
            PlanningConfig::Instance().maneuver_target_lane_forward_clear_threshold()
            || target_following_clear_distance
                < PlanningConfig::Instance().maneuver_target_lane_backward_clear_threshold()) {

          // cannot change lane, because the clear distance is too short
          maneuver_goal->maneuver_infos.resize(1);
          maneuver_goal->maneuver_infos.front().lane_id = before_lane_id_;
          maneuver_goal->maneuver_infos.front().ptr_ref_line = before_reference_line_;
          maneuver_goal->maneuver_infos.front().maneuver_target.target_speed =
              leading_vehicle_id < 0 ? PlanningConfig::Instance().target_speed() :
              std::min(PlanningConfig::Instance().target_speed(),
                       ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());
          maneuver_goal->maneuver_infos.front().has_stop_point = false;
          maneuver_goal->decision_type = DecisionType::kFollowLane;
        } else {
          //  can continue change lane
          SLPoint ego_sl_target_lane;
          after_reference_line_->XYToSL(VehicleState::Instance().pose().position.x,
                                        VehicleState::Instance().pose().position.y,
                                        &ego_sl_target_lane);
          maneuver_goal->decision_type = DecisionType::kChangeRight;
          maneuver_goal->maneuver_infos.resize(2);
          maneuver_goal->maneuver_infos.front().lane_id = before_lane_id_;
          maneuver_goal->maneuver_infos.front().ptr_ref_line = before_reference_line_;
          maneuver_goal->maneuver_infos.front().has_stop_point = false;
          maneuver_goal->maneuver_infos.front().maneuver_target.target_speed =
              leading_vehicle_id < 0 ? PlanningConfig::Instance().target_speed() :
              std::min(PlanningConfig::Instance().target_speed(),
                       ObstacleFilter::Instance().Obstacles().at(leading_vehicle_id)->Speed());

          maneuver_goal->maneuver_infos.back().lane_id = after_lane_id_;
          maneuver_goal->maneuver_infos.back().ptr_ref_line = after_reference_line_;
          maneuver_goal->maneuver_infos.back().maneuver_target.target_speed =
              target_leading_vehicle_id < 0 ?
              PlanningConfig::Instance().target_speed() : std::min(PlanningConfig::Instance().target_speed(),
                                                                   ObstacleFilter::Instance().Obstacles().at(
                                                                       target_leading_vehicle_id)->Speed());
          maneuver_goal->maneuver_infos.back().has_stop_point = false;
        }
      }
    }
  }
}

}