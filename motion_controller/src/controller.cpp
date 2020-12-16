#include <derived_object_msgs/ObjectArray.h>
#include "controller.hpp"
#include "pid_controller/pid_controller.hpp"
#include "name/string_name.hpp"
#include "planning_msgs/Trajectory.h"
namespace control {

Controller::Controller(ros::NodeHandle &nh) : nh_(nh), vehicle_state_(std::make_unique<vehicle_state::VehicleState>()) {
  nh_.param<std::string>("/motion_control/controller_type", controller_type_, "pid");
  nh_.param<double>("/motion_control/pid_lookahead_time", pid_configs_.lookahead_time, 1.0);
  nh_.param<double>("/motion_control/lateral_pid_kp", pid_configs_.lat_configs.lat_kp, 1.95);
  nh_.param<double>("/motion_control/lateral_pid_kd", pid_configs_.lat_configs.lat_kd, 0.01);
  nh_.param<double>("/motion_control/lateral_pid_ki", pid_configs_.lat_configs.lat_ki, 1.4);
  nh_.param<double>("/motion_control/longitudinal_pid_kp", pid_configs_.lon_configs.lon_kp, 0.72);
  nh_.param<double>("/motion_control/longitudinal_pid_kd", pid_configs_.lon_configs.lon_kd, 0.2);
  nh_.param<double>("/motion_control/longitudinal_pid_ki", pid_configs_.lon_configs.lon_ki, 0.36);
  nh_.param<double>("/motion_control/loop_rate", loop_rate_, 50.0);
  if (controller_type_ == "pid") {
    control_strategy_ = std::make_unique<PIDController>(pid_configs_, loop_rate_);
  } else {
    ROS_FATAL("No such controller... [%s]", controller_type_.c_str());
    ROS_ASSERT(false);
  }

  trajectory_subscriber_ = nh_.subscribe<planning_msgs::Trajectory>(
      common::topic::kPublishedTrajectoryName,
      5, [this](const planning_msgs::Trajectory::ConstPtr &trajectory) {
        trajectory_ = *trajectory;
      });
  vehicle_info_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleInfo>(
      common::topic::kEgoVehicleInfoName, 5,
      [this](const carla_msgs::CarlaEgoVehicleInfo::ConstPtr &vehicle_info) {
        ego_vehicle_id_ = vehicle_info->id;
        ego_vehicle_info_ = *vehicle_info;
      });
  vehicle_status_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
      common::topic::kEgoVehicleStatusName, 5,
      [this](const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &vehicle_status) {
        this->ego_vehicle_status_ = *vehicle_status;
      });
  this->objects_subscriber_ = nh_.subscribe<derived_object_msgs::ObjectArray>(
      common::topic::kObjectsName, 5,
      [this](const derived_object_msgs::ObjectArray::ConstPtr &object_array) {
        this->objects_map_.clear();
        for (const auto &object : object_array->objects) {
          objects_map_.emplace(object.id, object);
        }
        ROS_INFO("the objects map_ size is: %lu", objects_map_.size());
      });

  carla_control_publisher_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>(
      common::topic::kEgoVehicleControlName, 1);
}

void Controller::RunOnce() {
  auto current_time_stamp = ros::Time::now();
  carla_msgs::CarlaEgoVehicleControl control;
  control.header.frame_id = "/map";
  control.header.stamp = current_time_stamp;

  if (ego_vehicle_id_ == -1) {
    Controller::EmergencyStopControl(control);
    carla_control_publisher_.publish(control);
    return;
  }
  if (objects_map_.find(ego_vehicle_id_) == objects_map_.end()) {
    Controller::EmergencyStopControl(control);
    carla_control_publisher_.publish(control);
    return;
  }
  if (trajectory_.status != planning_msgs::Trajectory::NORMAL){
    Controller::EmergencyStopControl(control);
    carla_control_publisher_.publish(control);
    return;
  }

  vehicle_state_->Update(ego_vehicle_status_, ego_vehicle_info_, objects_map_[ego_vehicle_id_]);
  if (!control_strategy_->Execute(current_time_stamp.toSec(), *vehicle_state_, trajectory_, control)) {
    Controller::EmergencyStopControl(control);
    carla_control_publisher_.publish(control);
    return;
  }
  control.steer *= -1.0;
  carla_control_publisher_.publish(control);

}

void Controller::EmergencyStopControl(carla_msgs::CarlaEgoVehicleControl &control) {
  control.brake = 1.0;
  control.steer = 0.0;
  control.hand_brake = false;
  control.manual_gear_shift = false;
  control.throttle = 0.0;
}

void Controller::Launch() {
  ros::WallRate loop_rate(loop_rate_);
  while (ros::ok()){
    ros::spinOnce();
    RunOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
}

}