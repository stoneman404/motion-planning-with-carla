#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROLLER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROLLER_HPP_

#include <ros/ros.h>
#include <unordered_map>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <planning_msgs/Trajectory.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include "control_config.hpp"
#include "control_strategy.hpp"
#include "vehicle_state/vehicle_state.hpp"

namespace control{
class Controller {
 public:
  explicit Controller(ros::NodeHandle& nh);
  void Launch();
  void RunOnce();
 private:
  static void EmergencyStopControl(carla_msgs::CarlaEgoVehicleControl& control);

 private:
  ros::NodeHandle nh_;
  ros::Publisher carla_control_publisher_;
  ros::Subscriber vehicle_info_subscriber_;
  ros::Subscriber vehicle_status_subscriber_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber ref_lane_subscriber_;
  ros::Subscriber objects_subscriber_;
  std::string controller_type_{"pid"};
  PIDConfigs pid_configs_;
  std::unique_ptr<ControlStrategy> control_strategy_;
  planning_msgs::Trajectory trajectory_;
  carla_msgs::CarlaEgoVehicleInfo ego_vehicle_info_;
  carla_msgs::CarlaEgoVehicleStatus ego_vehicle_status_;
  std::unique_ptr<vehicle_state::VehicleState> vehicle_state_;
  std::unordered_map<int, derived_object_msgs::Object> objects_map_;
  int ego_vehicle_id_ = -1;
  double loop_rate_{};



};


}


#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_CONTROLLER_HPP_
