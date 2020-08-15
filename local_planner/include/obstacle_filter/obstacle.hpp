
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_OBSTACLE_FILTER_OBSTACLE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_OBSTACLE_FILTER_OBSTACLE_HPP_
#include <list>
#include <memory>
#include <geometry_msgs/Point.h>
#include <planning_msgs/Trajectory.h>
#include <derived_object_msgs/ObjectArray.h>
#include <carla_msgs/CarlaActorInfo.h>

#include "box2d.hpp"
namespace planning {
class Obstacle {
public:
    Obstacle() = default;
    ~Obstacle() = default;
    explicit Obstacle(const derived_object_msgs::Object &object);
    Obstacle(const Obstacle &other);
//    void SetId(const int& id) { this->id_ = id;}
    void set_lane_id(const int &lane_id) { this->lane_id_ = lane_id; }
    void set_section_id(const int &section_id) { this->section_id_ = section_id; }
    void set_road_id(const int &road_id) { this->road_id_ = road_id; }

    planning_msgs::TrajectoryPoint GetPointAtTime(double relative_time) const;
    Box2d GetBoundingBox() const { return bounding_box_; }
    bool HasTrajectory() const { return !trajectory_.trajectory_points.empty(); }
    // getter
    const ros::Time &TimeStamp() const { return time_stamp_; }
    const derived_object_msgs::Object &Object() const { return object_; }
    const Eigen::Vector2d &Center() const { return center_; }
    const bool &IsStatic() const { return is_static_; }
    const double &Speed() const { return speed_; }
    const int &Id() const { return id_; }
    const int &road_id() const { return road_id_; }
    const int &section_id() const { return section_id_; }
    const int &lane_id() const { return lane_id_; }
    const bool &IsValidObstacle() const { return this->is_valid_obstacle_; }
    const planning_msgs::Trajectory &Trajectory() const { return this->trajectory_; }
    const double &AngularSpeed() const { return angular_speed_; }
    const double &Heading() const { return heading_; }
    const double &Length() const { return length_; }
    const double &Width() const { return width_; }
    const Box2d &BoundingBox() const { return bounding_box_; }
    Box2d GetBoundingBoxAtPoint(const planning_msgs::TrajectoryPoint &point);

private:
    int lane_id_ = -1;
    int road_id_ = -1;
    int section_id_ = -1;
    ros::Time time_stamp_;
    derived_object_msgs::Object object_;
    double heading_;
    Eigen::Vector2d center_;
    bool is_static_ = false;
    bool is_valid_obstacle_;
    planning_msgs::Trajectory trajectory_;
    int id_;
    double speed_;
    double angular_speed_;
    double length_;
    double width_;
    Box2d bounding_box_;
};
}
#endif
