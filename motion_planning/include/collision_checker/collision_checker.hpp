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
                   ThreadPool *thread_pool = nullptr);
  /**
   * @brief: check ego vehicle is collision with obstacles
   * @param: trajectory: ego vehicle's trajectory
   * @return: true if collision with a certain obstacle, false otherwise
   */
  bool IsCollision(const planning_msgs::Trajectory &trajectory) const;

 private:
  /**
   *
   * @param obstacles
   * @param ego_vehicle_s
   * @param ego_vehicle_d
   * @param reference_line
   */
  void Init(const std::unordered_map<int, std::shared_ptr<Obstacle>> &obstacles, double ego_vehicle_s,
            double ego_vehicle_d, const std::shared_ptr<ReferenceLine> &reference_line);

  /**
   *
   * @param ego_vehicle_s
   * @param ego_vehicle_d
   * @return
   */
  bool IsEgoVehicleInLane(double ego_vehicle_s, double ego_vehicle_d) const;

  /**
   * @param[in] obstacle
   * @param[in] ego_s: current ego_s;
   * @param[in] ref_line: reference line
   * @return
   */
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
