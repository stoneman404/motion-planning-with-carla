#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_KEEP_LANE_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_KEEP_LANE_STATE_HPP_
#include "state.hpp"
namespace planning {

class KeepLaneState : public State {
public:
    bool Enter(ManeuverPlanner* maneuver_planner) override;
    bool Execute(ManeuverPlanner* maneuver_planner) override;
    void Exit(ManeuverPlanner* maneuver_planner) override;
    std::string Name() const override {return "KeepLaneState";}
    static State& Instance();
    // get the cost, and decide the next station
    double TransitionFunction();
private:
    KeepLaneState() = default;
    KeepLaneState(const KeepLaneState &other);
    KeepLaneState &operator=(const KeepLaneState &other);

};
}
#endif
