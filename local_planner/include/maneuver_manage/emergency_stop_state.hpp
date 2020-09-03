#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_EMERGENCY_STOP_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_EMERGENCY_STOP_HPP_
#include "state.hpp"

namespace planning {
class EmergencyStopState : public State {
public:
    bool Enter(ManeuverPlanner *maneuver_planner) override;
    void Exit(ManeuverPlanner *maneuver_planner) override;
    bool Execute(ManeuverPlanner *maneuver_planner) override;
    static State &Instance();
    std::string Name() const override;
    State* NextState(ManeuverPlanner* maneuver_planner) const override ;
private:
    EmergencyStopState() = default;
    EmergencyStopState(const EmergencyStopState &other);
    EmergencyStopState &operator=(const EmergencyStopState &other);
};
}
#endif
