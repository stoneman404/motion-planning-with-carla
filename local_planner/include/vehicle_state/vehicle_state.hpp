
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_VEHICLE_STATE_VEHICLE_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_VEHICLE_STATE_VEHICLE_STATE_HPP_
#include <ros/ros.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <planning_msgs/WayPoint.h>
#include "box2d.hpp"

namespace planning {
class VehicleState {
 public:

  static planning::VehicleState &Instance();
  bool Update(const carla_msgs::CarlaEgoVehicleStatus &ego_vehicle_status,
              const nav_msgs::Odometry &odometry,
              const carla_msgs::CarlaEgoVehicleInfo &vehicle_info);
  // getter
  Box2d GetEgoBox() const;
  const ros::Time &time_stamp() const;
  const geometry_msgs::Pose &pose() const;
  double heading() const;
  double linear_vel() const;
  double angular_vel() const;
  double linear_acc() const;
  double centripential_acc() const;
  double steer_percentage() const;
  geometry_msgs::Vector3 center_of_mass() const;
  const int &lane_id() const;
  const int &road_id() const { return road_id_; }
  const int &section_id() const { return section_id_; }
  const planning_msgs::WayPoint &ego_waypoint() const;
  // setter
  void set_waypoint(const planning_msgs::WayPoint &way_point);
  void set_lane_id(int lane_id);
  void set_section_id(int section_id);
  void set_road_id(int road_id);
  void set_linear_vel(double vel);
  void set_angular_vel(double omega);
  void set_pose(const geometry_msgs::Pose &pose);
  void set_linear_acc(double acc);
  void set_center_of_mass(const geometry_msgs::Vector3 &center_of_mass);
  // predict the vehicle's last pose based on current pose, linear/angular acc and vel after time t;
  geometry_msgs::Pose PredictNextPose(double t) const;
  bool is_junction() const;
  void set_is_junction(bool is_junction);

 private:

  planning_msgs::WayPoint ego_waypoint_;

  int lane_id_ = -1;
  int section_id_ = -1;
  int road_id_ = -1;
  bool is_junction_ = false;
  geometry_msgs::Pose pose_;
//    double kappa_;
  double heading_{};
  double linear_vel_{};
  double angular_vel_{};
  double linear_acc_{};
  double centripental_acc_{};
  double steer_percentage_{};
  bool reverse_ = false;
  geometry_msgs::Vector3 center_of_mass_;
  ros::Time time_stamp_;

 private:
  VehicleState() = default;
  ~VehicleState() = default;
};
}

#endif
