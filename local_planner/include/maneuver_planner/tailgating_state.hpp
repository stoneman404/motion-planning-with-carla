
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_TAILGATING_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_TAILGATING_STATE_HPP_
#include "state.hpp"
namespace planning {
class TailgatingState : public State {
 public:
  bool Enter(ManeuverPlanner *maneuver_planner) override;
  bool Execute(ManeuverPlanner *maneuver_planner) override;
  void Exit(ManeuverPlanner *maneuver_planner) override;
  std::string Name() const override;
  static State &Instance();
  State *NextState(ManeuverPlanner *maneuver_planner) const override;

 private:
  TailgatingState() = default;
  TailgatingState(const TailgatingState &other);
  TailgatingState &operator=(const TailgatingState &other);
};

}
#endif //
