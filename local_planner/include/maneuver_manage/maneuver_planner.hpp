#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_

// this is a simpler version of behavior planner,

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <Eigen/Core>
#include <vector>
#include "state.hpp"
#include "vehicle_state/vehicle_state.hpp"

namespace planning {
class State;
class ManeuverPlanner {
public:
    ManeuverPlanner();
    ~ManeuverPlanner(){};
    void InitPlanner();
    bool Process();
    void SetState(State& new_state);
    std::shared_ptr<State> GetState() const;
    int GetLaneId() const {return current_lane_id_;}
    void UpdateLocalGoal();


private:
    std::shared_ptr<State> current_state_;
    int current_lane_id_;


};
};
#endif //
