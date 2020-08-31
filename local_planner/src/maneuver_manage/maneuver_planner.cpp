#include "maneuver_manage/maneuver_planner.hpp"
#include "maneuver_manage/state.hpp"
#include "maneuver_manage/keep_lane_state.hpp"
#include "maneuver_manage/change_lane_left_state.hpp"
#include "maneuver_manage/change_lane_right_state.hpp"
#include "maneuver_manage/stop_at_sign.hpp"
#include "maneuver_manage/emergency_stop_state.hpp"
namespace planning {

ManeuverPlanner::ManeuverPlanner() {
  current_lane_id_ =
      VehicleState::Instance().lane_id();
  this->InitPlanner();
}

void ManeuverPlanner::InitPlanner() {
  this->current_state_.reset(&KeepLaneState::Instance());
}

void ManeuverPlanner::SetState(State &new_state) {
  this->current_state_->Exit(this);
  this->current_state_.reset(&new_state);
  current_state_->Enter(this);
}

bool ManeuverPlanner::Process() {

  return true;
}

}