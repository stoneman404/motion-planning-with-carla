#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_STATE_HPP_
#include "maneuver_planner.hpp"
#include <memory>
namespace planning {
class ManeuverPlanner;
class State {

 public:
  State() = default;
  virtual bool Enter(ManeuverPlanner *maneuver_planner) = 0;
  virtual bool Execute(ManeuverPlanner *maneuver_planner) = 0;
  virtual void Exit(ManeuverPlanner *maneuver_planner) = 0;
  virtual std::string Name() const = 0;
  virtual State *NextState(ManeuverPlanner *maneuver_planner) const = 0;
  virtual ~State() = default;

};

}

#endif
