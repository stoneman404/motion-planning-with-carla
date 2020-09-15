#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_STATE_HPP_
#include "maneuver_planner.hpp"
#include <memory>
namespace planning{
class ManeuverPlanner;
enum class StateName : uint32_t {
    kKeepLane = 1u,
    kEmergencyStop = 2u,
    kChangeLaneLeft = 3u,
    kChangeLaneRight = 4u,
    kStopAtSign = 5u
};

class State{

public:
    State() = default;
    virtual bool Enter(ManeuverPlanner* maneuver_planner) = 0;
    virtual bool Execute(ManeuverPlanner* maneuver_planner) = 0;
    virtual void Exit(ManeuverPlanner* maneuver_planner) = 0;
    virtual std::string Name() const = 0;
    virtual State* NextState(ManeuverPlanner* maneuver_planner) const = 0;
    virtual ~State() = default;
    virtual std::vector<StateName> GetPosibileNextStates() const = 0;

};

}



#endif
