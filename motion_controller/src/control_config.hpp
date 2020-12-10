
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROL_CONFIG_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROL_CONFIG_HPP_
namespace control {
struct LonPIDConfigs{
  double lon_kp{};
  double lon_kd{};
  double lon_ki{};
};

struct LatPIDConfigs{
  double lat_kp{};
  double lat_kd{};
  double lat_ki{};
};

struct PIDConfigs{
  double pid_lookahead_distance{};
  LonPIDConfigs lon_configs;
  LatPIDConfigs lat_configs;
};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROL_CONFIG_HPP_
