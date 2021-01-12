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
  using State = std::array<double, 4>;
  IntelligentDriverModel() = default;
  ~IntelligentDriverModel() = default;
  IntelligentDriverModel(const IDMParams &param, std::array<double, 4> &state);
  std::array<double, 4> Step(double dt);
  void operator()(const State &x, State &dxdt, const double /*t*/);

 private:
  static bool GetAccDesiredAcceleration(const IDMParams &param,
                                        const State &cur_state, double *acc);
  static bool GetIIdmDesiredAcceleration(const IDMParams &param,
                                         const State &cur_state, double *acc);
 private:
  IDMParams param_;
  State state_;
};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_FORWARD_SIMULATOR_INTELLIGENT_VELOCITY_CONTROL_HPP_
