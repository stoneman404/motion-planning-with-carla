
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_NORMAL_STOP_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_NORMAL_STOP_STATE_HPP_
#include "state.hpp"

namespace planning{
class StopAtSign : public State{
public:
    void Enter(ManeuverPlanner* manuever_planner) override ;
    bool Execute(ManeuverPlanner* manuever_planner) override ;
    void Exit(ManeuverPlanner* manuvever_planner) override ;
    static State& Instance();
private:
    StopAtSign() = default;
    StopAtSign(const StopAtSign& other) ;
    StopAtSign& operator=(const StopAtSign& other);

};
}
#endif //
