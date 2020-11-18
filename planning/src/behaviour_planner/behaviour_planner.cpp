
#include "behaviour_planner/behaviour_planner.hpp"

namespace planning {
BehaviourPlanner::BehaviourPlanner(const ros::NodeHandle &nh, const std::string &type, common::ThreadPool *thread_pool)
    : nh_(nh), thread_pool_(thread_pool) {
  behavior_.lon_behaviour_ = LongitudinalBehaviour::kStopping;
  behavior_.lat_behaviour_ = LateralBehaviour::kLaneKeeping;
  behavior_.desired_velocity_ = 0.0;
  behavior_.forward_behaviours_ = decltype(behavior_.forward_behaviours_)();
//  behavior_.surround_trajs_ = decltype(behavior_.surround_trajs_)();
  if (type == "mpdm") {
    // mpdm planner
//    behaviour_strategy_ = std::make_unique<MPDMPlanner>();
  } else if (type == "eudm") {
    // eudm planner
//    behaviour_strategy_ = std::make_unique<MPDMPlanner>();
  } else if (type == "fsm") {
    // finite state machine planner
  } else {

  }
  behaviour_strategy_ = std::make_unique<MPDMPlanner>();

}

bool BehaviourPlanner::Process(const planning_msgs::TrajectoryPoint &init_point, Behaviour &behaviour) {
  if (behaviour_strategy_ == nullptr) {
    ROS_FATAL("BehaviourPlanner, Not Init BehaviourStrategy");
    return false;
  }
  behaviour_strategy_->Execute(init_point, behaviour);
  return true;
}
}

