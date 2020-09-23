#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_VEHICLE_PARAMS_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_VEHICLE_PARAMS_HPP_
#include <Eigen/Core>
namespace planning{
struct VehicleParams {
  double length;
  double width;
  double back_axle_to_center_length;
  double front_axle_to_center_length;
  double axle_length_;
  double max_steer_angle_;
  double min_r_;
  Eigen::Vector3d center_of_mass_; // x, y , z
};



}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_VEHICLE_PARAMS_HPP_
