
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROL_CONFIG_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROL_CONFIG_HPP_
namespace control {
struct LonPIDConfigs {
  double lon_kp{};
  double lon_kd{};
  double lon_ki{};
};

struct LatPIDConfigs {
  double lat_kp{};
  double lat_kd{};
  double lat_ki{};
};

struct PurePursuitConfigs {
  double steer_control_gain = 1.5;
  double steer_control_max_lookahead_dist = 50.0;
  double steer_control_min_lookahead_dist = 3.0;
};

struct ControlConfigs {
  double lookahead_time{};
  LonPIDConfigs lon_configs;
  PurePursuitConfigs pure_pursuit_configs;
//  LatPIDConfigs lat_configs;
};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROL_CONFIG_HPP_
