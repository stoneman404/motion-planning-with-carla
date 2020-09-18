#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_KEEP_LANE_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_KEEP_LANE_STATE_HPP_
#include "state.hpp"
#include <vector>
#include "change_lane_right_state.hpp"
#include "change_lane_left_state.hpp"
#include "stop_at_sign.hpp"
#include "emergency_stop_state.hpp"

namespace planning {

class KeepLaneState : public State {

 public:
  bool Enter(ManeuverPlanner *maneuver_planner) override;
  bool Execute(ManeuverPlanner *maneuver_planner) override;
  void Exit(ManeuverPlanner *maneuver_planner) override;
  std::string Name() const override;
  static State &Instance();
  State *NextState(ManeuverPlanner *maneuver_planner) const override;
  std::vector<StateName> GetPosibileNextStates() const override;

 private:

  bool CurrentLaneProhibitedByObstacles() const;
  bool CurrentLaneProhibitedByTrafficLight() const;
  static bool WithInDistanceAhead(double target_x,
                                  double target_y,
                                  double current_x,
                                  double current_y,
                                  double heading,
                                  double max_distance);
  KeepLaneState() = default;
  KeepLaneState(const KeepLaneState &other);
  KeepLaneState &operator=(const KeepLaneState &other);
 private:
  std::shared_ptr<ReferenceLine> reference_line_;

};
}
#endif
