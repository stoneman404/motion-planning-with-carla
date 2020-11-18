#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_MPDM_BEHAVIOUR_PLANNER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_MPDM_BEHAVIOUR_PLANNER_HPP_
#include "behaviour_planner/behaviour_strategy.hpp"
namespace planning {
class MPDMPlanner : public BehaviourStrategy {
 public:
  MPDMPlanner() = default;
  ~MPDMPlanner() override = default;
  bool Execute(const planning_msgs::TrajectoryPoint& init_point, Behaviour& behaviour) override;
};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNING_INCLUDE_BEHAVIOUR_PLANNING_BEHAVIOUR_PLANNER_HPP_
