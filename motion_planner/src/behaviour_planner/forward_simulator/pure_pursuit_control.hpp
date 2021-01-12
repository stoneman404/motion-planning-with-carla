#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_FORWARD_SIMULATOR_PURE_PURSUIT_CONTROL_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_FORWARD_SIMULATOR_PURE_PURSUIT_CONTROL_HPP_
#include <cmath>
#include <array>
#include "vehicle_state/kinodynamic_state.hpp"

namespace planning {



class PurePursuitControl {
 public:
  static bool CalculateDesiredSteer(double wheel_base_len,
                                    double angle_diff,
                                    double lookahead_dist,
                                    double *steer);
};

class IdealSteerModel {
 public:

  struct Control {
    double steer{0.0};
    double velocity{0.0};
    Control() = default;
    Control(double s, double v) : steer(s), velocity(v) {}
  };

  IdealSteerModel(double wheelbase_len, double max_lon_acc, double max_lon_dec,
                  double max_lon_acc_jerk, double max_lon_dec_jerk,
                  double max_lat_acc, double max_lat_jerk,
                  double max_steering_angle, double max_steer_rate,
                  double max_curvature);
  ~IdealSteerModel() = default;
  vehicle_state::KinoDynamicState Step(const Control& control, const vehicle_state::KinoDynamicState& state, double dt);
  void operator()(const std::array<double, 5>& x, std::array<double, 5>& dxdt, const double /*t*/);
 private:
  void ClampControl(double dt);

 private:
  vehicle_state::KinoDynamicState state_;
  Control control_;
  double desired_steer_rate_{};
  double desired_lon_acc_{};
  double desired_lat_acc_{};
  double wheelbase_len_;
  double max_lon_acc_;
  double max_lon_decel_;
  double max_lon_acc_jerk_;
  double max_lon_decel_jerk_;
  double max_lat_acc_;
  double max_lat_jerk_;
  double max_steering_;
  double max_steer_rate_;
  double max_curvature_;
};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_FORWARD_SIMULATOR_PURE_PURSUIT_CONTROL_HPP_
