#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_FORWARD_SIMULATOR_INTELLIGENT_VELOCITY_CONTROL_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_FORWARD_SIMULATOR_INTELLIGENT_VELOCITY_CONTROL_HPP_
#include <boost/numeric/odeint.hpp>
#include "intelligent_velocity_control.hpp"
#include "behaviour_planner/behaviour_configs/behaviour_configs.hpp"
namespace planning {

class IntelligentVelocityControl {
 public:
  static bool CalculateDesiredVelocity(const IDMParams &params, const double s,
                                       const double s_front, const double v,
                                       const double v_front, const double dt,
                                       double *velocity_at_dt);
};

class IntelligentDriverModel {
 public:
  using std::array<double, 4>
  State;
  IntelligentDriverModel() = default;
  ~IntelligentDriverModel() = default;
  IntelligentDriverModel(const IDMParams &param, std::array<double, 4> &state);
 private:
  State state_;
  IDMParams param_;
};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_FORWARD_SIMULATOR_INTELLIGENT_VELOCITY_CONTROL_HPP_
