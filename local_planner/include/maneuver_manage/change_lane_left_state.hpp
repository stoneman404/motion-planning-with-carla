
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LANE_LEFT_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LANE_LEFT_HPP_
#include "state.hpp"
namespace planning{
class ChangeLaneLeft : public State{
public:
    void Enter(ManeuverPlanner* manuever_planner) override ;
    bool Execute(ManeuverPlanner* manuever_planner) override ;
    void Exit(ManeuverPlanner* manuever_planner) override ;
    static State& Instance();

private:
    ChangeLaneLeft() = default;
    ChangeLaneLeft(const ChangeLaneLeft& other);
    ChangeLaneLeft operator=(const ChangeLaneLeft& other);
};

}
#endif //
