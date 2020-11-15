#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_COMMON_VEHICLE_PARAMS_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_COMMON_VEHICLE_PARAMS_HPP_
#include <Eigen/Core>
namespace vehicle_state{
struct VehicleParams {
  double length;
  double width;
  double half_length;
  double half_width;
  double back_axle_to_center_length;
  double front_axle_to_center_length;
  double axle_length_;
  double max_steer_angle_;
  double min_r_;
  double lr_; // back axle to center of mass
  double lf_; // front axle to center of mass
//  Eigen::Vector3d center_of_mass_; // x, y , z
};



}
#endif
