#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_BEHAVIOUR_PLANNER_BEHAVIOUR_STRATEGY_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_BEHAVIOUR_PLANNER_BEHAVIOUR_STRATEGY_HPP_
#include <planning_msgs/TrajectoryPoint.h>
#include <agent/behaviour.hpp>
#include <agent/agent.hpp>
namespace planning {


class BehaviourStrategy {

 public:
  BehaviourStrategy() = default;
  virtual ~BehaviourStrategy() = default;
  virtual void SetAgentSet(const std::unordered_map<int, Agent>& agent_set) = 0;
  virtual bool Execute(const planning_msgs::TrajectoryPoint &init_point, Behaviour &behaviour) = 0;
};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_INCLUDE_BEHAVIOUR_PLANNER_BEHAVIOUR_STRATEGY_HPP_
