
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_OBSTACLE_FILTER_OBSTACLE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_OBSTACLE_FILTER_OBSTACLE_HPP_
#include <list>
#include <memory>
#include <geometry_msgs/Point.h>
#include <planning_msgs/Trajectory.h>
#include <derived_object_msgs/ObjectArray.h>
#include <carla_msgs/CarlaActorInfo.h>
#include <carla_msgs/CarlaTrafficLightStatus.h>
#include <carla_msgs/CarlaTrafficLightInfo.h>
#include "polygon/box2d.hpp"

namespace planning {
class Obstacle {
 public:
  Obstacle() = default;
  ~Obstacle() = default;
  explicit Obstacle(const derived_object_msgs::Object &object);

  Obstacle(const carla_msgs::CarlaTrafficLightInfo &traffic_light_info,
           const carla_msgs::CarlaTrafficLightStatus &traffic_light_status);
  Obstacle(const Obstacle &other);
  void PredictTrajectory(double predict_horizon, double predict_step);

  void SetTrajectory(const planning_msgs::Trajectory &trajectory);
  const planning_msgs::Trajectory &trajectory() const { return trajectory_; }
  planning_msgs::TrajectoryPoint GetPointAtTime(double relative_time) const;
  const common::Box2d &GetBoundingBox() const;
  bool HasTrajectory() const;

  // getter
  double x() const { return center_.x(); }
  double y() const { return center_.y(); }

  const double &acc() const { return acc_; }
  const double &kappa() const { return kappa_; }
  const double &centripental_acc() const { return centripental_acc_; }
  const Eigen::Vector2d &Center() const;
  const bool &IsStatic() const;
  const double &Speed() const;
  const int &Id() const;
  const bool &IsVirtual() const { return is_virtual_; }
  const bool &IsValidObstacle() const;
  const planning_msgs::Trajectory &GetPredictedTrajectory() const;
  const double &AngularSpeed() const;
  const double &Heading() const;
  const common::Box2d &BoundingBox() const;
  common::Box2d GetBoundingBoxAtPoint(const planning_msgs::TrajectoryPoint &point) const;

 private:
  int id_{};
  double acc_{};
  double centripental_acc_{};
  double kappa_{};
  double heading_{};
  Eigen::Vector2d center_{};
  bool is_static_ = false;
  bool is_virtual_ = false;
  bool is_valid_obstacle_{};
  planning_msgs::Trajectory trajectory_;
  double speed_{};
  double angular_speed_{};

  common::Box2d bounding_box_;

};
}
#endif
