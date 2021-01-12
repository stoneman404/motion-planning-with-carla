#include "on_lane_forward_simulator.hpp"
#include "math/coordinate_transformer.hpp"
#include "planning_config.hpp"
namespace planning {

bool OnLaneForwardSimulator::ForwardOneStep(const Agent &agent,
                                            const SimulationParams &params,
                                            const ReferenceLine &reference_line,
                                            const Agent &leading_agent,
                                            double cur_relative_time,
                                            double sim_time_step,
                                            planning_msgs::TrajectoryPoint &point) {
  params_ = params;
  double wheel_base = 0.0;
  if (agent.is_host()) {
    wheel_base = PlanningConfig::Instance().vehicle_params().axle_length_;
  } else {
    wheel_base = agent.bounding_box().length() * 0.585;
  }
  double approx_lookahead_dist =
      std::min(std::max(params_.steer_control_min_lookahead_dist, agent.state().v * params_.steer_control_gain),
               params_.steer_control_max_lookahead_dist);
  double steer = 0.0;
  auto ego_state = agent.state();
  auto leading_state = agent.state();
  bool has_shift_ego_state = false;
  if (!agent.is_rear_centered()) {
    double shift_dis = 0.5 * wheel_base;
    ego_state.ShiftState({-std::cos(ego_state.theta) * shift_dis,
                          -std::sin(ego_state.theta) * shift_dis});
    has_shift_ego_state = true;
  }

  CalculateSteer(reference_line, ego_state, wheel_base, {approx_lookahead_dist, 0.0}, &steer);
  double sim_vel = params_.idm_params.desired_velocity;
  params_.idm_params.desired_velocity = std::max(0.0, sim_vel);
  double velocity = 0.0;
  if (!leading_agent.is_valid()) {
    CalculateVelocityUsingIdm(params_, agent.state().v, sim_time_step, &velocity);
  } else {
    common::SLPoint sl_point, leading_sl_point;
    if (!reference_line.XYToSL(ego_state.x, ego_state.y, &sl_point)) {
      return false;
    }
    if (!reference_line.XYToSL(leading_state.x, leading_state.y, &leading_sl_point)) {
      return false;
    }
    //todo here the position should be in rear axle center
    CalculateVelocityUsingIdm(params_, sl_point.s, agent.state().v,
                              leading_sl_point.s, leading_agent.state().v,
                              sim_time_step, &velocity);
  }
  vehicle_state::KinoDynamicState state{};
  CalculateDesiredState(params, ego_state, steer, velocity, wheel_base, sim_time_step, &state);
  // recetered at agent geometric center.
  if (!agent.is_rear_centered() && has_shift_ego_state) {
    double shift_dis = 0.5 * wheel_base;
    ego_state.ShiftState({std::cos(ego_state.theta) * shift_dis,
                          std::sin(ego_state.theta) * shift_dis});
  }
  point = state.ToTrajectoryPoint(sim_time_step + cur_relative_time);
  return true;
}

bool OnLaneForwardSimulator::CalculateSteer(const ReferenceLine &reference_line,
                                            const vehicle_state::KinoDynamicState &cur_state,
                                            double wheelbase_len,
                                            const std::array<double, 2> &lookahead_offset,
                                            double *steer) {
  Eigen::Vector2d cur_xy{cur_state.x, cur_state.y};
  common::SLPoint sl_point;
  if (reference_line.XYToSL(cur_xy, &sl_point)) {
    return false;
  }
  common::SLPoint dest_sl;
  dest_sl.s = sl_point.s + lookahead_offset[0];
  dest_sl.l = lookahead_offset[1];
  Eigen::Vector2d destination;
  if (reference_line.SLToXY(dest_sl, &destination)) {
    return false;
  }
  double lookahead_dist = (destination - cur_xy).norm();
  double cur_to_dest_angle = common::MathUtils::NormalizeAngle(
      std::atan2(destination.y() - cur_xy.y(),
                 destination.x() - cur_xy.x()));
  double angle_diff = common::MathUtils::CalcAngleDist(cur_state.theta, cur_to_dest_angle);
  PurePursuitControl::CalculateDesiredSteer(wheelbase_len, angle_diff, lookahead_dist, steer);
  return true;

}
bool OnLaneForwardSimulator::CalculateVelocityUsingIdm(const SimulationParams &param,
                                                       double cur_s,
                                                       double cur_v,
                                                       double leading_s,
                                                       double leading_v,
                                                       double dt,
                                                       double *velocity) {
  double leading_vel_fin = leading_v;
  if (leading_vel_fin < 0) {
    leading_vel_fin = 0.0;
  }
  return IntelligentVelocityControl::CalculateDesiredVelocity(param.idm_params,
                                                              cur_s,
                                                              leading_s,
                                                              cur_v,
                                                              leading_vel_fin,
                                                              dt,
                                                              velocity);
}

bool OnLaneForwardSimulator::CalculateVelocityUsingIdm(const SimulationParams &param,
                                                       double cur_v,
                                                       double dt,
                                                       double *velocity) {
  const double virtual_leading_dist = 100.0 + 100.0 * cur_v;
  return IntelligentVelocityControl::CalculateDesiredVelocity(
      param.idm_params, 0.0, 0.0 + virtual_leading_dist, cur_v,
      cur_v, dt, velocity);
}
bool OnLaneForwardSimulator::CalculateDesiredState(const SimulationParams &param,
                                                   const vehicle_state::KinoDynamicState &cur_state,
                                                   double steer,
                                                   double velocity,
                                                   double wheel_base,
                                                   double dt,
                                                   vehicle_state::KinoDynamicState *state) {
  IdealSteerModel model(wheel_base, param.idm_params.max_acc, param.idm_params.max_decel,
                        param.max_lon_acc_jerk,
                        param.max_lon_brake_jerk,
                        param.max_lat_acceleration_abs,
                        param.max_lat_jerk_abs, param.max_steer_angle_abs,
                        param.max_steer_rate, param.max_curvature_abs);
  auto control = IdealSteerModel::Control(steer, velocity);
  *state = model.Step(control, cur_state, dt);
  return true;
}
}