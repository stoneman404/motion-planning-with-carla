#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_FORWARD_SIMULATOR_ON_LANE_FORWARD_SIMULATOR_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_FORWARD_SIMULATOR_ON_LANE_FORWARD_SIMULATOR_HPP_
#include <reference_line/reference_line.hpp>
#include <behaviour_planner/agent/behaviour.hpp>
#include <behaviour_planner/agent/agent.hpp>
#include "pure_pursuit_control.hpp"
#include "intelligent_velocity_control.hpp"
#include "behaviour_planner/behaviour_configs/behaviour_configs.hpp"
namespace planning {

class OnLaneForwardSimulator {
 public:
  OnLaneForwardSimulator() = default;
  ~OnLaneForwardSimulator() = default;
  bool ForwardOneStep(const Agent &agent,
                      const SimulationParams &params,
                      const ReferenceLine &reference_line,
                      const Agent &leading_agent,
                      double cur_relative_time,
                      double sim_time_step,
                      planning_msgs::TrajectoryPoint &point);

 private:
  static bool CalculateSteer(const ReferenceLine &reference_line,
                             const vehicle_state::KinoDynamicState &cur_state,
                             double wheelbase_len,
                             const std::array<double, 2> &lookahead_offset,
                             double *steer);

  static bool CalculateVelocityUsingIdm(const SimulationParams &param, double cur_s, double cur_v,
                                        double leading_s, double leading_v, double dt,
                                        double *velocity);

  static bool CalculateVelocityUsingIdm(const SimulationParams &param, double cur_v,
                                        double dt, double *velocity);

  static bool CalculateDesiredState(const SimulationParams &param,
                                    const vehicle_state::KinoDynamicState &cur_state,
                                    double steer, double velocity,
                                    double wheel_base,
                                    double dt, vehicle_state::KinoDynamicState *state);


 private:
  SimulationParams params_;

};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_FORWARD_SIMULATOR_ON_LANE_FORWARD_SIMULATOR_HPP_
