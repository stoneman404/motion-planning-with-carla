#include <tf/transform_datatypes.h>
#include "obstacle_manager/obstacle.hpp"
#include "math/math_utils.hpp"

namespace planning {
using namespace common;
Obstacle::Obstacle(const derived_object_msgs::Object &object) {
  this->is_valid_obstacle_ = (!std::isnan(object.shape.dimensions[0])
      && !std::isnan(object.shape.dimensions[1])
      && !std::isnan(object.shape.dimensions[2]));
  id_ = object.id;
  this->speed_ = std::sqrt(object.twist.linear.x +
      object.twist.linear.y + object.twist.linear.z);
  this->angular_speed_ = object.twist.angular.z;
  this->is_static_ = std::fabs(this->speed_) < 0.1 && std::fabs(this->angular_speed_) < 0.1;
  this->heading_ = tf::getYaw(object.pose.orientation);
  center_ = Eigen::Vector2d(object.pose.position.x, object.pose.position.y);
  this->bounding_box_ = Box2d(center_, heading_, object.shape.dimensions[0], object.shape.dimensions[1]);
  acc_ = object.accel.linear.x * std::cos(heading_) + object.accel.linear.y * std::sin(heading_);
  centripental_acc_ = object.accel.linear.y * std::cos(heading_) - object.accel.linear.x * std::sin(heading_);
  kappa_ = std::fabs(speed_) < 1e-2 ? 0.0 : angular_speed_ / speed_;
}

Obstacle::Obstacle(const Obstacle &other) {
  this->id_ = other.id_;
  this->bounding_box_ = other.bounding_box_;
  this->speed_ = other.speed_;
  this->angular_speed_ = other.angular_speed_;
  this->trajectory_ = other.trajectory_;
  this->is_valid_obstacle_ = other.is_valid_obstacle_;
  this->is_static_ = other.is_static_;
  this->center_ = other.center_;
  this->heading_ = other.heading_;
}

Box2d Obstacle::GetBoundingBoxAtPoint(const planning_msgs::TrajectoryPoint &point) const {
  Eigen::Vector2d center = Eigen::Vector2d(point.path_point.x, point.path_point.y);
  return Box2d(center, point.path_point.theta, bounding_box_.length(), bounding_box_.width());
}

planning_msgs::TrajectoryPoint Obstacle::GetPointAtTime(double relative_time) const {
  const auto &trajectory_points = trajectory_.trajectory_points;
  // for static obstacle
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

const Box2d &Obstacle::GetBoundingBox() const { return bounding_box_; }

const Eigen::Vector2d &Obstacle::Center() const { return center_; }

const bool &Obstacle::IsStatic() const { return is_static_; }

const double &Obstacle::Speed() const { return speed_; }

const int &Obstacle::Id() const { return id_; }

const planning_msgs::Trajectory &Obstacle::GetPredictedTrajectory() const { return this->trajectory_; }

const bool &Obstacle::IsValidObstacle() const { return this->is_valid_obstacle_; }

const Box2d &Obstacle::BoundingBox() const { return bounding_box_; }

const double &Obstacle::Heading() const { return heading_; }

const double &Obstacle::AngularSpeed() const { return angular_speed_; }

void Obstacle::PredictTrajectory(double predict_horizon, double predict_step) {
  if (is_static_) {
    trajectory_.trajectory_points.emplace_back();
    auto &trajectory_point = trajectory_.trajectory_points.back();
    trajectory_point.path_point.x = center_.x();
    trajectory_point.path_point.y = center_.y();
    trajectory_point.path_point.s = 0.0;
    trajectory_point.path_point.theta = heading_;
    trajectory_point.path_point.kappa = 0.0;
    trajectory_point.path_point.dkappa = 0.0;
    trajectory_point.relative_time = 0.0;
    trajectory_point.vel = this->speed_;
    trajectory_point.acc = acc_;
    trajectory_point.jerk = 0.0;
    trajectory_point.steer_angle = 0.0;
  } else {
    planning_msgs::TrajectoryPoint last_trajectory_point;
    last_trajectory_point.path_point.x = this->center_.x();
    last_trajectory_point.path_point.y = this->center_.y();
    last_trajectory_point.path_point.kappa = 0.0;
    last_trajectory_point.path_point.dkappa = 0.0;
    last_trajectory_point.path_point.s = 0.0;
    last_trajectory_point.path_point.theta = heading_;
    last_trajectory_point.vel = speed_;
    last_trajectory_point.acc = acc_;
    last_trajectory_point.jerk = 0.0;
    last_trajectory_point.steer_angle = 0.0;
    last_trajectory_point.relative_time = 0.0;
    trajectory_.trajectory_points.push_back(last_trajectory_point);
    const int kStepSize =
        static_cast<int>(predict_horizon / predict_step) < 1 ? 1 : static_cast<int>(predict_horizon / predict_step);
    for (int i = 1; i < kStepSize; ++i) {
      trajectory_.trajectory_points.emplace_back();
      auto &trajectory_point = trajectory_.trajectory_points.back();
      trajectory_point.path_point.x = last_trajectory_point.path_point.x +
          predict_step * last_trajectory_point.vel * std::cos(last_trajectory_point.path_point.theta);
      trajectory_point.path_point.y = last_trajectory_point.path_point.y +
          predict_step * last_trajectory_point.vel * std::sin(last_trajectory_point.path_point.theta);
      trajectory_point.path_point.theta = last_trajectory_point.path_point.theta;
      trajectory_point.path_point.kappa = 0.0;
      trajectory_point.path_point.dkappa = 0.0;
      trajectory_point.relative_time = predict_step * static_cast<double>(i);
      trajectory_point.vel = speed_;
      trajectory_point.acc = acc_;
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

Obstacle::Obstacle(const carla_msgs::CarlaTrafficLightInfo &traffic_light_info,
                   const carla_msgs::CarlaTrafficLightStatus &traffic_light_status)
    : id_(traffic_light_info.id), acc_(0.0), centripental_acc_(0.0),
      kappa_(0.0), is_static_(true), is_virtual_(true), trajectory_(planning_msgs::Trajectory()),
      speed_(0.0), angular_speed_(0.0) {
  is_valid_obstacle_ = false;
  if (traffic_light_status.state == carla_msgs::CarlaTrafficLightStatus::RED
      || traffic_light_status.state == carla_msgs::CarlaTrafficLightStatus::YELLOW) {
    is_valid_obstacle_ = true;
  }
  double box_length = traffic_light_info.trigger_volume.size.x;
  double box_width = traffic_light_info.trigger_volume.size.y;
  center_ <<  traffic_light_info.trigger_volume.center.x, traffic_light_info.trigger_volume.center.y;
  heading_ = tf::getYaw(traffic_light_info.transform.orientation);
  bounding_box_ = common::Box2d(center_, heading_, box_length, box_width);
}

void Obstacle::SetTrajectory(const planning_msgs::Trajectory &trajectory) {
  this->trajectory_ = trajectory;
}

}
