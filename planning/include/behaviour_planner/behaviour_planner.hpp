
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNER_INCLUDE_BEHAVIOUR_PLANNER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNER_INCLUDE_BEHAVIOUR_PLANNER_HPP_
#include <string>
#include <unordered_map>
#include <memory>
#include <planning_msgs/TrajectoryPoint.h>
#include <planning_msgs/Trajectory.h>
#include <thread_pool/thread_pool.hpp>
#include "behaviour_planner/behaviour_strategy.hpp"
#include "mpdm_planner/mpdm_planner.hpp"

namespace planning {

class BehaviourPlanner {
 public:
  BehaviourPlanner() = default;
  ~BehaviourPlanner() = default;
  BehaviourPlanner(const ros::NodeHandle &nh, const std::string &type, common::ThreadPool *thread_pool = nullptr);
  bool Process(const planning_msgs::TrajectoryPoint &init_point, Behaviour &behaviour);
 private:
  ros::NodeHandle nh_;
  common::ThreadPool *thread_pool_;
  Behaviour behavior_;
  std::unique_ptr<BehaviourStrategy> behaviour_strategy_;

};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNER_INCLUDE_BEHAVIOUR_PLANNER_HPP_
