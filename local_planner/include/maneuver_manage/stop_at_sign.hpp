
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_NORMAL_STOP_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_NORMAL_STOP_STATE_HPP_
#include "state.hpp"

namespace planning{
class StopAtSignState : public State{
public:
    bool Enter(ManeuverPlanner* manuever_planner) override ;
    bool Execute(ManeuverPlanner* manuever_planner) override ;
    void Exit(ManeuverPlanner* manuvever_planner) override ;
    std::string Name() const override {return "StopAtSignState";}
    static State& Instance();
private:
    StopAtSignState() = default;
    StopAtSignState(const StopAtSignState& other) ;
    StopAtSignState& operator=(const StopAtSignState& other);

};
}
#endif //
