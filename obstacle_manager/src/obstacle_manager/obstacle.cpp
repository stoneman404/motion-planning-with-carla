#include <tf/transform_datatypes.h>
#include "obstacle_manager/obstacle.hpp"
#include "math/math_utils.hpp"

namespace planning {
using namespace common;
Obstacle::Obstacle(const derived_object_msgs::Object &object) {
  this->is_valid_obstacle_ = (!std::isnan(object_.shape.dimensions[0])
      && !std::isnan(object_.shape.dimensions[0])
      && !std::isnan(object_.shape.dimensions[2]));
  this->time_stamp_ = object.header.stamp;
  object_ = object;
  id_ = object_.id;
  this->speed_ = std::sqrt(object_.twist.linear.x +
      object_.twist.linear.y + object_.twist.linear.z);
  this->angular_speed_ = object_.twist.angular.z;
  this->is_static_ = std::fabs(this->speed_) < 0.1
      && std::fabs(this->angular_speed_) < 0.1;
  this->length_ = object_.shape.dimensions[0];
  this->width_ = object_.shape.dimensions[1];
  this->heading_ = tf::getYaw(object_.pose.orientation);
  center_ = Eigen::Vector2d(object_.pose.position.x, object_.pose.position.y);
  this->bounding_box_ = Box2d(center_, heading_, length_, width_);
  trajectory_.trajectory_points.clear();
  trajectory_.header = object.header;
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(object_.pose.orientation, quaternion);
  tf::Matrix3x3 R = tf::Matrix3x3(quaternion);
  tf::Vector3 global_acc;
  global_acc.setX(object_.accel.linear.x);
  global_acc.setY(object_.accel.linear.y);
  global_acc.setZ(object_.accel.linear.z);
  tf::Vector3 body_acc = R * global_acc;

  if (is_static_) {
    trajectory_.trajectory_points.emplace_back();
    auto &trajectory_point = trajectory_.trajectory_points.back();
    trajectory_point.path_point.x = object.pose.position.x;
    trajectory_point.path_point.y = object.pose.position.y;
    trajectory_point.path_point.s = 0.0;
    trajectory_point.path_point.theta = heading_;
    trajectory_point.path_point.kappa = 0.0;
    trajectory_point.path_point.dkappa = 0.0;
    trajectory_point.relative_time = 0.0;
    trajectory_point.vel = this->speed_;
    trajectory_point.acc = body_acc.x();
    trajectory_point.jerk = 0.0;
    trajectory_point.steer_angle = 0.0;
  } else {
    double trajectory_time_length = 8.0;
    double delta_t = 0.1;
    planning_msgs::TrajectoryPoint last_trajectory_point;
    last_trajectory_point.path_point.x = this->object_.pose.position.x;
    last_trajectory_point.path_point.y = this->object_.pose.position.y;
    last_trajectory_point.path_point.kappa = 0.0;
    last_trajectory_point.path_point.dkappa = 0.0;
    last_trajectory_point.path_point.s = 0.0;
    last_trajectory_point.path_point.theta = heading_;
    last_trajectory_point.vel = speed_;
    last_trajectory_point.acc = body_acc.x();
    last_trajectory_point.jerk = 0.0;
    last_trajectory_point.steer_angle = 0.0;
    last_trajectory_point.relative_time = 0.0;
    trajectory_.trajectory_points.push_back(last_trajectory_point);
    for (double t = delta_t; t <= trajectory_time_length; t += delta_t) {
      trajectory_.trajectory_points.emplace_back();
      auto &trajectory_point = trajectory_.trajectory_points.back();
      trajectory_point.path_point.x = last_trajectory_point.path_point.x +
          delta_t * last_trajectory_point.vel * std::cos(last_trajectory_point.path_point.theta);
      trajectory_point.path_point.y = last_trajectory_point.path_point.y +
          delta_t * last_trajectory_point.vel * std::sin(last_trajectory_point.path_point.theta);
      trajectory_point.path_point.theta = last_trajectory_point.path_point.theta;
      trajectory_point.path_point.kappa = 0.0;
      trajectory_point.path_point.dkappa = 0.0;
      trajectory_point.relative_time = t;
      trajectory_point.vel = speed_;
      trajectory_point.acc = body_acc.x();
      trajectory_point.jerk = 0.0;
      trajectory_point.steer_angle = 0.0;
      double dx = trajectory_point.path_point.x - last_trajectory_point.path_point.x;
      double dy = trajectory_point.path_point.y - last_trajectory_point.path_point.y;
      double delta_s = std::hypot(dx, dy);
      trajectory_point.path_point.s = last_trajectory_point.path_point.s + delta_s;
      last_trajectory_point = trajectory_point;
    }
  }
}

Obstacle::Obstacle(const Obstacle &other) {
  this->id_ = other.id_;
  this->lane_id_ = other.lane_id_;
  this->section_id_ = other.section_id_;
  this->bounding_box_ = other.bounding_box_;
  this->speed_ = other.speed_;
  this->length_ = other.length_;
  this->width_ = other.width_;
  this->angular_speed_ = other.angular_speed_;
  this->object_ = other.object_;
  this->trajectory_ = other.trajectory_;
  this->is_valid_obstacle_ = other.is_valid_obstacle_;
  this->is_static_ = other.is_static_;
  this->center_ = other.center_;
  this->time_stamp_ = other.time_stamp_;
  this->heading_ = other.heading_;
}

Box2d Obstacle::GetBoundingBoxAtPoint(const planning_msgs::TrajectoryPoint &point) const {
  Eigen::Vector2d center = Eigen::Vector2d(point.path_point.x, point.path_point.y);
  return Box2d(center, point.path_point.theta, length_, width_);
}

planning_msgs::TrajectoryPoint Obstacle::GetPointAtTime(double relative_time) const {
  const auto &trajectory_points = trajectory_.trajectory_points;
  if (trajectory_points.size() < 2) {
    planning_msgs::TrajectoryPoint point;
    point.path_point.s = 0.0;
    point.path_point.x = center_[0];
    point.path_point.y = center_[1];
    point.path_point.theta = heading_;
    point.path_point.kappa = 0.0;
    point.path_point.dkappa = 0.0;
    point.vel = 0.0;
    point.acc = 0.0;
    point.jerk = 0.0;
    point.steer_angle = 0.0;
    point.relative_time = 0.0;
    return point;
  } else {
    auto comp = [](const planning_msgs::TrajectoryPoint &p, const double &relative_time) -> bool {
      return p.relative_time < relative_time;
    };
    auto it_lower = std::lower_bound(trajectory_points.begin(),
                                     trajectory_points.end(), relative_time, comp);
    if (it_lower == trajectory_points.begin()) {
      return *trajectory_points.begin();
    }
    if (it_lower == trajectory_points.end()) {
      return *trajectory_points.rbegin();
    }
    return MathUtils::InterpolateTrajectoryPoint(*(it_lower - 1), *it_lower, relative_time);
  }
}

bool Obstacle::HasTrajectory() const { return trajectory_.trajectory_points.size() > 1; }

void Obstacle::set_lane_id(const int &lane_id) { this->lane_id_ = lane_id; }

void Obstacle::set_section_id(const int &section_id) { this->section_id_ = section_id; }

void Obstacle::set_road_id(const int &road_id) { this->road_id_ = road_id; }

const Box2d & Obstacle::GetBoundingBox() const { return bounding_box_; }

const ros::Time &Obstacle::TimeStamp() const { return time_stamp_; }

const derived_object_msgs::Object &Obstacle::Object() const { return object_; }

const Eigen::Vector2d &Obstacle::Center() const { return center_; }

const bool &Obstacle::IsStatic() const { return is_static_; }

const double &Obstacle::Speed() const { return speed_; }

const int &Obstacle::Id() const { return id_; }

const int &Obstacle::road_id() const { return road_id_; }

const int &Obstacle::section_id() const { return section_id_; }

const int &Obstacle::lane_id() const { return lane_id_; }

const double &Obstacle::Length() const { return length_; }

const double &Obstacle::Width() const { return width_; }

const planning_msgs::Trajectory &Obstacle::Trajectory() const { return this->trajectory_; }

const bool &Obstacle::IsValidObstacle() const { return this->is_valid_obstacle_; }

const Box2d &Obstacle::BoundingBox() const { return bounding_box_; }

const double &Obstacle::Heading() const { return heading_; }

const double &Obstacle::AngularSpeed() const { return angular_speed_; }

}
