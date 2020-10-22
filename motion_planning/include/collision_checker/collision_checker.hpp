#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_COLLISION_CHECKER_COLLISION_CHECKER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_COLLISION_CHECKER_COLLISION_CHECKER_HPP_
#include "reference_line/reference_line.hpp"
#include "ros/ros.h"
#include "box2d.hpp"
#include <planning_msgs/Trajectory.h>
#include "st_graph.hpp"
#include "thread_pool.hpp"

namespace planning {
class CollisionChecker {
 public:
  CollisionChecker() = default;
  ~CollisionChecker() = default;
  CollisionChecker(const std::shared_ptr<ReferenceLine> &ptr_ref_line,
                   const std::shared_ptr<STGraph> &ptr_st_graph,
                   double ego_vehicle_s, double ego_vehicle_d,
                   ThreadPool *thread_pool);
  bool IsCollision(const planning_msgs::Trajectory &trajectory) const;
 private:
  void Init(const std::unordered_map<int, std::shared_ptr<Obstacle>> &obstacles, double ego_vehicle_s,
            double ego_vehicle_d, const std::shared_ptr<ReferenceLine> &reference_line);
  bool IsEgoVehicleInLane(double ego_vehicle_s, double ego_vehicle_d) const;
  bool IsObstacleBehindEgoVehicle(const std::shared_ptr<Obstacle> &obstacle, double ego_s,
                                  const std::shared_ptr<ReferenceLine> &ref_line) const;
 private:
  std::shared_ptr<ReferenceLine> ptr_ref_line_;
  std::shared_ptr<STGraph> ptr_st_graph_;
  std::vector<std::vector<Box2d>> predicted_obstacle_box_;
  ThreadPool *thread_pool_ = nullptr;

};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_COLLISION_CHECKER_COLLISION_CHECKER_HPP_
