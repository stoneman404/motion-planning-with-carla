#include "obstacle_filter/obstacle_filter.hpp"
#include "traffic_lights/traffic_light_list.hpp"
#include "maneuver_planner/follow_lane.hpp"
#include "maneuver_planner/emergency_stop.hpp"
#include "maneuver_planner/stop.hpp"
#include "maneuver_planner/change_left_lane.hpp"
#include "maneuver_planner/change_right_lane.hpp"
namespace planning {

bool EmergencyStop::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("Oops, something going wrong, we enter the **EmergencyStop**");
}

void EmergencyStop::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are exiting **EmergencyStop**");
}

State &EmergencyStop::Instance() {
  static EmergencyStop instance;
  return instance;
}
std::string EmergencyStop::Name() const { return "EmergencyStop"; }

State *EmergencyStop::Transition(ManeuverPlanner *maneuver_planner) {
  if (maneuver_planner == nullptr) {
    return nullptr;
  }
  if (maneuver_planner->prev_maneuver_status() == ManeuverStatus::kUnknown ||
      maneuver_planner->prev_maneuver_status() == ManeuverStatus::kError) {
    ManeuverGoal maneuver_goal;
    maneuver_goal.maneuver_infos.resize(1);
    maneuver_goal.decision_type = DecisionType::kEmergencyStop;
    maneuver_goal.maneuver_infos.front().has_stop_point = true;
    maneuver_planner->SetManeuverGoal(maneuver_goal);
    return &(EmergencyStop::Instance());
  }
  auto init_trajectory_point = maneuver_planner->init_trajectory_point();
  reference_line_ = maneuver_planner->multable_ref_line().front();
  SLPoint ego_sl;
  reference_line_->XYToSL(init_trajectory_point.path_point.x,
                          init_trajectory_point.path_point.y,
                          &ego_sl);
  reference_line_->NearestWayPoint(ego_sl.s);
  current_lane_id_ = reference_line_->NearestWayPoint(ego_sl.s + 5.0).lane_id;
  ManeuverGoal obstacle_maneuver;
  ManeuverGoal traffic_light_maneuver;
  this->ObstacleDecision(init_trajectory_point, &obstacle_maneuver);
  this->TrafficLightDecision(reference_line_, ego_sl, &traffic_light_maneuver);
  auto combined_maneuver = CombineManeuver(traffic_light_maneuver, obstacle_maneuver);
  switch (combined_maneuver.decision_type) {
    case DecisionType::kFollowLane: return &(FollowLane::Instance());
    case DecisionType::kEmergencyStop: return &(EmergencyStop::Instance());
    case DecisionType::kStopAtTrafficSign:
    case DecisionType::kStopAtDestination: return &(Stop::Instance());
    case DecisionType::kChangeRight: return &(ChangeRightLane::Instance());
    case DecisionType::kChangeLeft: return &(ChangeLeftLane::Instance());
    default:return nullptr;
  }
}

void EmergencyStop::ObstacleDecision(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                     ManeuverGoal *maneuver_goal) const {
  double forward_clear_distance, backward_clear_distance;
  int leading_vehicle_id, following_vehicle_id;
  SLPoint ego_sl;
  reference_line_->XYToSL(init_trajectory_point.path_point.x,
                          init_trajectory_point.path_point.y,
                          &ego_sl);
  Eigen::Vector2d ego_center{init_trajectory_point.path_point.x, init_trajectory_point.path_point.y};
  const double ego_theta = init_trajectory_point.path_point.theta;
  const double ego_length = PlanningConfig::Instance().vehicle_params().length;
  const double ego_width = PlanningConfig::Instance().vehicle_params().width;
  SLBoundary sl_boundary;
  Box2d ego_box = Box2d(ego_center, ego_theta, ego_length, ego_width);
  reference_line_->GetSLBoundary(ego_box, &sl_boundary);
  this->GetLaneClearDistance(0, sl_boundary,
                             reference_line_,
                             &forward_clear_distance,
                             &backward_clear_distance,
                             &leading_vehicle_id,
                             &following_vehicle_id);
  maneuver_goal->maneuver_infos.resize(1);
  if (forward_clear_distance > PlanningConfig::Instance().lon_safety_buffer()
      && backward_clear_distance > PlanningConfig::Instance().lon_safety_buffer()) {
    // following lane
    maneuver_goal->decision_type = DecisionType::kFollowLane;
    maneuver_goal->maneuver_infos.front().has_stop_point = false;
    maneuver_goal->maneuver_infos.front().maneuver_target.target_speed =
        leading_vehicle_id < 0 ?
        PlanningConfig::Instance().target_speed() : ObstacleFilter::Instance().Obstacles().at(
            leading_vehicle_id)->Speed();
    maneuver_goal->maneuver_infos.front().lane_id = current_lane_id_;
    maneuver_goal->maneuver_infos.front().ptr_ref_line = reference_line_;
  } else {
    // emergency stop
    maneuver_goal->decision_type = DecisionType::kEmergencyStop;
    maneuver_goal->maneuver_infos.front().has_stop_point = true;
    maneuver_goal->maneuver_infos.front().maneuver_target.target_s =
        ego_sl.s + PlanningConfig::Instance().lon_safety_buffer();
    maneuver_goal->maneuver_infos.front().lane_id = current_lane_id_;
    maneuver_goal->maneuver_infos.front().ptr_ref_line = reference_line_;
  }
}

}