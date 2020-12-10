#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROL_STRATEGY_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROL_STRATEGY_HPP_
#include <ros/ros.h>
#include "vehicle_state/vehicle_state.hpp"
namespace control {
class ControlStrategy {
 public:
  ControlStrategy() = default;
  virtual ~ControlStrategy() = default;
  virtual bool Execute(double current_time_stamp,
                       const vehicle_state::VehicleState &vehicle_state,
                       const planning_msgs::Trajectory &trajectory,
                       carla_msgs::CarlaEgoVehicleControl &control) = 0;
 protected:

};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROL_STRATEGY_HPP_
