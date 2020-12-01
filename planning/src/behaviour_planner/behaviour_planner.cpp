#include "behaviour_planner.hpp"
#include "mpdm_planner/mpdm_planner.hpp"
#include <memory>

namespace planning {

BehaviourPlanner::BehaviourPlanner(const ros::NodeHandle &nh,
                                   const std::string &type,
                                   common::ThreadPool *thread_pool) {

  if (type == "mpdm") {
    behaviour_strategy_ = std::make_unique<MPDMPlanner>(nh, thread_pool);
  } else {
    ROS_ERROR("No such type BehaviourPlanner, [%s]", type.c_str());
    ROS_ASSERT(false);
  }
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

