
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNER_INCLUDE_BEHAVIOUR_PLANNER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNER_INCLUDE_BEHAVIOUR_PLANNER_HPP_
#include <string>
#include <unordered_map>
#include <memory>
#include <planning_msgs/TrajectoryPoint.h>
#include <planning_msgs/Trajectory.h>
#include <thread_pool/thread_pool.hpp>
#include <planning_config.hpp>
#include <tf/transform_datatypes.h>
#include "behaviour_strategy.hpp"

namespace planning {

class BehaviourPlanner {
 public:
  BehaviourPlanner() = default;
  ~BehaviourPlanner() = default;
  BehaviourPlanner(const ros::NodeHandle &nh, const std::string &type, common::ThreadPool *thread_pool = nullptr);
  bool Process(const planning_msgs::TrajectoryPoint &init_point, Behaviour &behaviour);
 private:
  std::unique_ptr<BehaviourStrategy> behaviour_strategy_;
};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNER_INCLUDE_BEHAVIOUR_PLANNER_HPP_
