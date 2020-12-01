#include "vehicle_state/vehicle_state.hpp"
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <polygon/box2d.hpp>

namespace vehicle_state {
// the vehicle's coordinate, x--->forward, y-->left, z--->up

VehicleState::VehicleState(const carla_msgs::CarlaEgoVehicleStatus &ego_vehicle_status,
                           const nav_msgs::Odometry &odometry,
                           const carla_msgs::CarlaEgoVehicleInfo &vehicle_info,
                           const derived_object_msgs::Object &object) {

  this->vehicle_params_.length = object.shape.dimensions[0];
  this->vehicle_params_.width = object.shape.dimensions[1];
  this->vehicle_params_.half_length = vehicle_params_.length / 2.0;
  this->vehicle_params_.half_width = vehicle_params_.width / 2.0;
  this->vehicle_params_.back_axle_to_center_length = std::fabs(vehicle_info.wheels[3].position.x);
  this->vehicle_params_.front_axle_to_center_length = std::fabs(vehicle_info.wheels[0].position.x);
  this->vehicle_params_.axle_length_ = this->vehicle_params_.front_axle_to_center_length +
      this->vehicle_params_.back_axle_to_center_length;
  this->vehicle_params_.max_steer_angle_ = vehicle_info.wheels.front().max_steer_angle;
  this->vehicle_params_.min_r_ = this->vehicle_params_.axle_length_ / std::tan(this->vehicle_params_.max_steer_angle_);
  this->vehicle_params_.lr_ = std::fabs(vehicle_info.wheels[3].position.x - vehicle_info.center_of_mass.x);
  this->vehicle_params_.lf_ = std::fabs(vehicle_info.wheels[0].position.x - vehicle_info.center_of_mass.y);
  double ego_theta = tf::getYaw(odometry.pose.pose.orientation);
  double ego_x = odometry.pose.pose.position.x - vehicle_params_.back_axle_to_center_length * std::cos(ego_theta);
  double ego_y = odometry.pose.pose.position.y - vehicle_params_.back_axle_to_center_length * std::sin(ego_theta);
  this->time_stamp_ = ego_vehicle_status.header.stamp;
  // 我们假设车辆的滑移角很小，为零 所以车速朝向和车头朝向一致，
  double ego_v = ego_vehicle_status.velocity;
  double ego_omega = odometry.twist.twist.angular.z;
  double ego_kappa = 0.0;
  if (ego_v < 1e-6) {
    ego_kappa = 0.0;
  } else {
    ego_kappa = ego_omega / ego_v;
  }

  double ego_a = ego_vehicle_status.acceleration.linear.x * std::cos(ego_theta) + ego_vehicle_status.acceleration.linear.y * std::sin(ego_theta);
  double centripental_acc = -1.0 * ego_vehicle_status.acceleration.linear.x * std::sin(ego_theta) + ego_vehicle_status.acceleration.linear.y * std::cos(ego_theta);
  kino_dynamic_state_ = KinoDynamicState(ego_x, ego_y, odometry.pose.pose.position.z, ego_theta, ego_kappa, ego_v, ego_a, centripental_acc);

  ROS_INFO("centripental_acc is %lf", centripental_acc);
  this->time_stamp_ = ego_vehicle_status.header.stamp;
  this->steer_percentage_ = ego_vehicle_status.control.steer;
  this->reverse_ = ego_vehicle_status.control.reverse;
  this->id_ = object.id;
}

const ros::Time &VehicleState::time_stamp() const { return this->time_stamp_; }

void VehicleState::Update(const carla_msgs::CarlaEgoVehicleStatus &ego_vehicle_status,
                          const nav_msgs::Odometry &odometry,
                          const carla_msgs::CarlaEgoVehicleInfo &vehicle_info,
                          const derived_object_msgs::Object &object) {
  this->vehicle_params_.length = object.shape.dimensions[0];
  this->vehicle_params_.width = object.shape.dimensions[1];
  this->vehicle_params_.half_length = vehicle_params_.length / 2.0;
  this->vehicle_params_.half_width = vehicle_params_.width / 2.0;
  this->vehicle_params_.back_axle_to_center_length = std::fabs(vehicle_info.wheels[3].position.x);
  this->vehicle_params_.front_axle_to_center_length = std::fabs(vehicle_info.wheels[0].position.x);
  this->vehicle_params_.axle_length_ = this->vehicle_params_.front_axle_to_center_length + this->vehicle_params_.back_axle_to_center_length;
  this->vehicle_params_.max_steer_angle_ = vehicle_info.wheels.front().max_steer_angle;
  this->vehicle_params_.min_r_ = this->vehicle_params_.axle_length_ / std::tan(this->vehicle_params_.max_steer_angle_);
  this->vehicle_params_.lr_ = std::fabs(vehicle_info.wheels[3].position.x - vehicle_info.center_of_mass.x);
  this->vehicle_params_.lf_ = std::fabs(vehicle_info.wheels[0].position.x - vehicle_info.center_of_mass.y);
  double ego_theta = tf::getYaw(odometry.pose.pose.orientation);

  double ego_x = odometry.pose.pose.position.x - vehicle_params_.back_axle_to_center_length * std::cos(ego_theta);
  double ego_y = odometry.pose.pose.position.y - vehicle_params_.back_axle_to_center_length * std::sin(ego_theta);
  this->time_stamp_ = ego_vehicle_status.header.stamp;
  // 我们假设车辆的滑移角很小，为零 所以车速朝向和车头朝向一致，
  double ego_v = ego_vehicle_status.velocity;
  double ego_omega = odometry.twist.twist.angular.z;
  double ego_kappa = 0.0;
  if (ego_v < 1e-6) {
    ego_kappa = 0.0;
  } else {
    ego_kappa = ego_omega / ego_v;
  }
  double ego_a = ego_vehicle_status.acceleration.linear.x * std::cos(ego_theta) + ego_vehicle_status.acceleration.linear.y * std::sin(ego_theta);
  double centripental_acc = -1.0 * ego_vehicle_status.acceleration.linear.x * std::sin(ego_theta) + ego_vehicle_status.acceleration.linear.y * std::cos(ego_theta);
  kino_dynamic_state_ =
      KinoDynamicState(ego_x, ego_y, odometry.pose.pose.position.z, ego_theta, ego_kappa, ego_v, ego_a, centripental_acc);
  ROS_INFO("centripental_acc is %lf", centripental_acc);
  this->time_stamp_ = ego_vehicle_status.header.stamp;
  this->steer_percentage_ = ego_vehicle_status.control.steer;
  this->reverse_ = ego_vehicle_status.control.reverse;
//  this->center_of_mass_ = vehicle_info.center_of_mass;
}



double VehicleState::steer_percentage() const { return steer_percentage_; }

bool VehicleState::is_junction() const { return is_junction_; }
void VehicleState::set_is_junction(bool is_junction) { this->is_junction_ = is_junction; }
const planning_msgs::WayPoint &VehicleState::ego_waypoint() const { return ego_waypoint_; }
const int &VehicleState::lane_id() const { return lane_id_; }
void VehicleState::set_lane_id(int lane_id) { this->lane_id_ = lane_id; }
void VehicleState::set_waypoint(const planning_msgs::WayPoint &way_point) { this->ego_waypoint_ = way_point; }
void VehicleState::set_road_id(int road_id) { this->road_id_ = road_id; }
void VehicleState::set_section_id(int section_id) { this->section_id_ = section_id; }
common::Box2d VehicleState::GetEgoBox() const {
  return ego_box_;
}
const int &VehicleState::section_id() const { return section_id_; }

const int &VehicleState::road_id() const { return road_id_; }

const KinoDynamicState & VehicleState::GetKinoDynamicVehicleState() const {
  return kino_dynamic_state_;
}

void VehicleState::PredictNextKinoDynamicState(double predict_time, KinoDynamicState *predicted_state) const {
  *predicted_state = kino_dynamic_state_.GetNextStateAfterTime(predict_time);
}
const int &VehicleState::id() const { return id_; }

}
