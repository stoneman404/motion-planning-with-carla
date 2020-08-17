#include <maneuver_manage/emergency_stop_state.hpp>
#include <obstacle_filter/obstacle_filter.hpp>
#include "maneuver_manage/keep_lane_state.hpp"
namespace planning {
void KeepLaneState::Enter(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are current switching to KeepLaneState");
}
bool KeepLaneState::Execute(ManeuverPlanner *maneuver_planner) {
  if (maneuver_planner == nullptr) {
    ROS_ERROR("the ManeuverPlanner is nullptr");
    return false;
  }

  if (VehicleState::Instance().lane_id() == -1
      || VehicleState::Instance().section_id() == -1
      || VehicleState::Instance().road_id() == -1) {
    ROS_ERROR("the ego vehicle is currently out of lane");
    maneuver_planner->SetState(EmergencyStopState::Instance());
    return true;
  }
  auto obstacles = ObstacleFilter::Instance().Obstacles();

  for (const auto &obstacle : obstacles) {
    // ignore the vehicle and pedestrians in other lane or road
    if (obstacle->road_id() != VehicleState::Instance().road_id()
        || obstacle->section_id() != VehicleState::Instance().section_id()
        || obstacle->lane_id() != VehicleState::Instance().lane_id()) {
      continue;
    }

  }

}
void KeepLaneState::Exit(ManeuverPlanner *maneuver_planner) {
  ROS_INFO("We are currently Exiting the KeepLaneState");
}
State &KeepLaneState::Instance() {
  static KeepLaneState instance;
  return instance;
}

double KeepLaneState::TransitionFunction() {

}

}
