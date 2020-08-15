#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LANE_RIGHT_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_CHANGE_LANE_RIGHT_STATE_HPP_
#include "state.hpp"
namespace planning{
class ChangeLaneRight : public State{
public:
    void Enter(ManeuverPlanner* manuever_planner) override ;
    bool Execute(ManeuverPlanner* manuever_planner) override ;
    void Exit(ManeuverPlanner* manuever_planner) override ;
    State& Instance();
private:
    ChangeLaneRight() = default;
    ChangeLaneRight(const ChangeLaneRight& other);
    ChangeLaneRight& operator=(const ChangeLaneRight& other);
};
}
#endif //
