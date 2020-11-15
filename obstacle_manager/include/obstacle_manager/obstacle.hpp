
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_OBSTACLE_FILTER_OBSTACLE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_OBSTACLE_FILTER_OBSTACLE_HPP_
#include <list>
#include <memory>
#include <geometry_msgs/Point.h>
#include <planning_msgs/Trajectory.h>
#include <derived_object_msgs/ObjectArray.h>
#include <carla_msgs/CarlaActorInfo.h>

#include "polygon/box2d.hpp"

namespace planning {
class Obstacle {
 public:
  Obstacle() = default;
  ~Obstacle() = default;
  explicit Obstacle(const derived_object_msgs::Object &object);
  Obstacle(const Obstacle &other);
//    void SetId(const int& id) { this->id_ = id;}
  void set_lane_id(const int &lane_id);
  void set_section_id(const int &section_id);
  void set_road_id(const int &road_id);

  planning_msgs::TrajectoryPoint GetPointAtTime(double relative_time) const;
  const common::Box2d & GetBoundingBox() const;
  bool HasTrajectory() const;

  // getter
  const ros::Time &TimeStamp() const;
  const derived_object_msgs::Object &Object() const;
  const Eigen::Vector2d &Center() const;
  const bool &IsStatic() const;
  const double &Speed() const;
  const int &Id() const;
  const int &road_id() const;
  const int &section_id() const;
  const int &lane_id() const;
  const bool &IsValidObstacle() const;
  const planning_msgs::Trajectory &Trajectory() const;
  const double &AngularSpeed() const;
  const double &Heading() const;
  const double &Length() const;
  const double &Width() const;
  const common::Box2d &BoundingBox() const;
  common::Box2d GetBoundingBoxAtPoint(const planning_msgs::TrajectoryPoint &point) const;
 private:

 private:
  int id_{};
  int lane_id_{};
  int road_id_{};
  int section_id_{};
  ros::Time time_stamp_{};
  derived_object_msgs::Object object_{};

  double heading_{};
  Eigen::Vector2d center_{};
  bool is_static_ = false;
  bool is_valid_obstacle_{};
  planning_msgs::Trajectory trajectory_;
  double speed_{};
  double angular_speed_{};
  double length_{};
  double width_{};
  common::Box2d bounding_box_;

};
}
#endif
