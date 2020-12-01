
#include "agent.hpp"
namespace planning {
Agent::Agent(const Obstacle &obstacle) : id_(obstacle.Id()),
                                         is_host_(false),
                                         is_valid_(true),
                                         bounding_box_(obstacle.GetBoundingBox()),
                                         max_lat_behaviour_(LateralBehaviour::kUndefined),
                                         current_ref_lane_(nullptr),
                                         target_ref_lane_(nullptr),
                                         has_trajectory_(false),
                                         trajectory_(planning_msgs::Trajectory()) {
  double x = obstacle.Center().x();
  double y = obstacle.Center().y();
  double z = obstacle.Object().pose.position.z;
  double theta = obstacle.Heading();
  double v = obstacle.Speed();
  double a = obstacle.acc();
  double kappa = obstacle.kappa();
  double centripental_acc = obstacle.centripental_acc();
  state_ = vehicle_state::KinoDynamicState(x, y, z, theta, kappa, v, a, centripental_acc);
}
Agent::Agent(const vehicle_state::VehicleState &vehicle_state) : id_(vehicle_state.id()),
                                                                 is_host_(true),
                                                                 is_valid_(true),
                                                                 bounding_box_(vehicle_state.GetEgoBox()),
                                                                 state_(vehicle_state.GetKinoDynamicVehicleState()),
                                                                 max_lat_behaviour_(LateralBehaviour::kUndefined),
                                                                 current_ref_lane_(nullptr),
                                                                 target_ref_lane_(nullptr),
                                                                 has_trajectory_(false),
                                                                 trajectory_(planning_msgs::Trajectory()) {

}
Agent::Agent() : id_(-1),
                 is_host_(false),
                 is_valid_(false),
                 bounding_box_(common::Box2d()),
                 way_point_(planning_msgs::WayPoint()),
                 state_(vehicle_state::KinoDynamicState()),
                 max_lat_behaviour_(LateralBehaviour::kUndefined),
                 current_ref_lane_(nullptr),
                 target_ref_lane_(nullptr),
                 has_trajectory_(false),
                 trajectory_(planning_msgs::Trajectory()) {}
const common::Box2d &Agent::bounding_box() const {
  return bounding_box_;
}
const planning_msgs::WayPoint &Agent::way_point() const { return way_point_; }
int Agent::id() const { return id_; }
const vehicle_state::KinoDynamicState &Agent::state() const {
  return state_;
}
bool Agent::is_valid() const {
  return is_valid_;
}
bool Agent::is_host() const {
  return is_host_;
}
const std::shared_ptr<ReferenceLine> &Agent::current_ref_lane() const {
  return current_ref_lane_;
}
const std::shared_ptr<ReferenceLine> &Agent::target_ref_lane() const {
  return target_ref_lane_;
}
const LateralBehaviour &Agent::most_likely_behaviour() const {
  return max_lat_behaviour_;
}
void Agent::set_current_ref_lane(const std::shared_ptr<ReferenceLine> &ref_line) {
  this->current_ref_lane_ = ref_line;
}
void Agent::set_target_ref_lane(const std::shared_ptr<ReferenceLine> &ref_lane) {
  this->target_ref_lane_ = ref_lane;
}
void Agent::set_trajectory(const planning_msgs::Trajectory &trajectory) {
  this->trajectory_ = trajectory;
}
void Agent::set_most_likely_behaviour(const LateralBehaviour &lateral_behaviour) {
  max_lat_behaviour_ = lateral_behaviour;
}
bool Agent::PredictAgentBehaviour() {
  common::FrenetFramePoint frame_point;
  planning_msgs::PathPoint path_point;
  StateToPathPoint(state_, path_point);
  auto frenet_point = current_ref_lane_->GetFrenetFramePoint(path_point);
  double prob_lcl = 0.0;
  double prob_lcr = 0.0;
  double prob_lk = 0.0;
  constexpr double kLatDistanceThreshold = 0.4;
  constexpr double kLatVelThreshold = 0.35;
  if (frenet_point.l > kLatDistanceThreshold && frenet_point.dl > kLatVelThreshold
      && current_ref_lane_->CanChangeLeft(frenet_point.s)) {
    prob_lcl = 1.0;
  } else if (frenet_point.l < -kLatDistanceThreshold && frenet_point.dl < -kLatVelThreshold
      && current_ref_lane_->CanChangeRight(frenet_point.s)) {
    prob_lcr = 1.0;
  } else {
    prob_lk = 1.0;
  }
  probs_lat_behaviour_.SetEntry(LateralBehaviour::kLaneChangeLeft, prob_lcl);
  probs_lat_behaviour_.SetEntry(LateralBehaviour::kLaneKeeping, prob_lk);
  probs_lat_behaviour_.SetEntry(LateralBehaviour::kLaneChangeRight, prob_lcr);
  probs_lat_behaviour_.GetMaxProbBehaviour(max_lat_behaviour_);

  return true;
}
void Agent::StateToPathPoint(const vehicle_state::KinoDynamicState &state, planning_msgs::PathPoint &path_point) {
  path_point.x = state.x_;
  path_point.y = state.y_;
  path_point.theta = state.theta_;
  path_point.kappa = state.kappa_;
  path_point.dkappa = 0.0;
}
}

