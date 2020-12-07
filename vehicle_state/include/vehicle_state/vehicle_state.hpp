#ifndef CATKIN_WS_MOTION_PLANNING_WITH_CARLA_VEHICLE_STATE_INCLUDE_VEHICLE_STATE_VEHICLE_STATE_HPP_
#define CATKIN_WS_MOTION_PLANNING_WITH_CARLA_VEHICLE_STATE_INCLUDE_VEHICLE_STATE_VEHICLE_STATE_HPP_
#include <ros/ros.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <planning_msgs/WayPoint.h>
#include <derived_object_msgs/Object.h>
#include "polygon/box2d.hpp"
#include "vehicle_state/vehicle_params.hpp"
#include "vehicle_state/kinodynamic_state.hpp"
namespace vehicle_state {
class VehicleState {
 public:

  VehicleState() = default;
  ~VehicleState() = default;

  VehicleState(const carla_msgs::CarlaEgoVehicleStatus &ego_vehicle_status,
               const carla_msgs::CarlaEgoVehicleInfo &vehicle_info,
               const derived_object_msgs::Object &object);

  void Update(const carla_msgs::CarlaEgoVehicleStatus &ego_vehicle_status,
              const carla_msgs::CarlaEgoVehicleInfo &vehicle_info,
              const derived_object_msgs::Object &object);
  // getter
  common::Box2d GetEgoBox() const;
  const ros::Time &time_stamp() const;
  double steer_percentage() const;
  const int &id() const;
//  const int &lane_id() const;
//  const int &road_id() const;
//  const int &section_id() const;
//  const planning_msgs::WayPoint &ego_waypoint() const;
  // setter
//  void set_waypoint(const planning_msgs::WayPoint &way_point);
//  void set_lane_id(int lane_id);
//  void set_section_id(int section_id);
//  void set_road_id(int road_id);
  void PredictNextKinoDynamicState(double predict_time, KinoDynamicState *predicted_state) const;
  const KinoDynamicState &GetKinoDynamicVehicleState() const;
  const vehicle_state::VehicleParams &vehicle_params() const { return vehicle_params_; }

  bool is_junction() const;
  void set_is_junction(bool is_junction);

 private:
  int id_{};
//  planning_msgs::WayPoint ego_waypoint_;
//  int lane_id_{};
//  int section_id_{};
//  int road_id_ = {};
  bool is_junction_ = false;
  double steer_percentage_{};
  bool reverse_ = false;
//  geometry_msgs::Vector3 center_of_mass_;
  ros::Time time_stamp_;
  vehicle_state::VehicleParams vehicle_params_{};
  KinoDynamicState kino_dynamic_state_{};
  common::Box2d ego_box_;
};
}
#endif //CATKIN_WS_MOTION_PLANNING_WITH_CARLA_VEHICLE_STATE_INCLUDE_VEHICLE_STATE_VEHICLE_STATE_HPP_
