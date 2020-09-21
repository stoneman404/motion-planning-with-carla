#include <tf/transform_datatypes.h>
#include "obstacle_filter/obstacle.hpp"
#include "planning_config.hpp"
#include "math_utils.hpp"

namespace planning {

Obstacle::Obstacle(const derived_object_msgs::Object &object) {
  this->is_valid_obstacle_ = !(std::isnan(object_.shape.dimensions[0])
      || std::isnan(object_.shape.dimensions[0])
      || std::isnan(object_.shape.dimensions[2]));
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
  tf::Quaternion quat;
  tf::quaternionMsgToTF(object_.pose.orientation, quat);
  tf::Matrix3x3 R = tf::Matrix3x3(quat);
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
    double trajectory_time_length = PlanningConfig::Instance().obstacle_trajectory_time();
    double delta_t = PlanningConfig::Instance().delta_t();
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
  this->id_ = other.Id();
  this->bounding_box_ = other.BoundingBox();
  this->speed_ = other.Speed();
  this->length_ = other.Length();
  this->width_ = other.Width();
  this->angular_speed_ = other.AngularSpeed();
  this->object_ = other.Object();
  this->trajectory_ = other.Trajectory();
  this->is_valid_obstacle_ = other.IsValidObstacle();
  this->is_static_ = other.IsStatic();
  this->center_ = other.Center();
  this->time_stamp_ = other.TimeStamp();
  this->heading_ = other.Heading();
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
    auto comp = [](const planning_msgs::TrajectoryPoint &p, const double &relative_time) {
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
    return InterpolateTrajectoryPoint(*(it_lower - 1), *it_lower, relative_time);
  }
}
planning_msgs::TrajectoryPoint Obstacle::InterpolateTrajectoryPoint(const planning_msgs::TrajectoryPoint &point,
                                                                    const planning_msgs::TrajectoryPoint &point_1,
                                                                    double time) {
  planning_msgs::TrajectoryPoint interpolated_traj_point;
  interpolated_traj_point.path_point.x =
      MathUtil::lerp(point.path_point.x, point.relative_time, point_1.path_point.x, point_1.relative_time, time);
  interpolated_traj_point.path_point.y =
      MathUtil::lerp(point.path_point.y, point.relative_time, point_1.path_point.y, point_1.relative_time, time);
  interpolated_traj_point.path_point.s =
      MathUtil::lerp(point.path_point.s, point.relative_time, point_1.path_point.s, point_1.relative_time, time);
  interpolated_traj_point.path_point.theta =
      MathUtil::slerp(point.path_point.theta, point.relative_time, point_1.path_point.theta, point_1.relative_time, time);
  interpolated_traj_point.path_point.kappa =
      MathUtil::lerp(point.path_point.kappa, point.relative_time, point_1.path_point.kappa, point_1.relative_time, time);
  interpolated_traj_point.path_point.dkappa =
      MathUtil::lerp(point.path_point.dkappa, point.relative_time, point_1.path_point.dkappa, point_1.relative_time, time);
  interpolated_traj_point.vel =
      MathUtil::lerp(point.vel, point.relative_time, point_1.vel, point_1.relative_time, time);
  interpolated_traj_point.acc =
      MathUtil::lerp(point.acc, point.relative_time, point_1.acc, point_1.relative_time, time);
  interpolated_traj_point.jerk =
      MathUtil::lerp(point.jerk, point.relative_time, point_1.jerk, point_1.relative_time, time);
  interpolated_traj_point.steer_angle =
      MathUtil::slerp(point.steer_angle, point.relative_time, point_1.steer_angle, point_1.relative_time, time);
  return interpolated_traj_point;







}

}
