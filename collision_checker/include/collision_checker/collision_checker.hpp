#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_COLLISION_CHECKER_COLLISION_CHECKER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_COLLISION_CHECKER_COLLISION_CHECKER_HPP_
#include "reference_line/reference_line.hpp"
#include "ros/ros.h"
#include "polygon/box2d.hpp"
#include <planning_msgs/Trajectory.h>
#include "obstacle_manager/st_graph.hpp"
#include "thread_pool/thread_pool.hpp"
#include "obstacle_manager/obstacle.hpp"
#include "vehicle_state/vehicle_params.hpp"

namespace planning {
class CollisionChecker {
 public:
  CollisionChecker() = default;
  ~CollisionChecker() = default;

  /**
   * @param obstacles: the nearby obstacles, which refers to vehicles, walkers and etc.
   * @param ptr_ref_line: the reference line
   * @param ptr_st_graph: the st graph to check
   * @param ego_vehicle_s: ego vehicle stational state
   * @param ego_vehicle_d: ego vehicle lateral state
   * @param lon_buffer: the safety buffer in lon direction
   * @param lat_buffer: the safety buffer in lat direction
   * @param lookahead_time: the max lookahead time, horizon
   * @param delta_t: the delta t to check collision along trajectory
   * @param vehicle_params: ego vehicle's params
   * @param thread_pool: the thread pool, to accelerate the calculation
   */
  CollisionChecker(const std::unordered_map<int, std::shared_ptr<Obstacle>> &obstacles,
                   std::shared_ptr<ReferenceLine> ptr_ref_line,
                   std::shared_ptr<STGraph> ptr_st_graph,
                   double ego_vehicle_s,
                   double ego_vehicle_d,
                   double lon_buffer,
                   double lat_buffer,
                   double lookahead_time,
                   double delta_t,
                   const vehicle_state::VehicleParams& vehicle_params,
                   common::ThreadPool *thread_pool);
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
  void Init(const std::unordered_map<int, std::shared_ptr<planning::Obstacle>> &obstacles, double ego_vehicle_s,
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
  bool IsObstacleBehindEgoVehicle(const std::shared_ptr<planning::Obstacle> &obstacle, double ego_s,
                                  const std::shared_ptr<ReferenceLine> &ref_line) const;

 private:
  std::shared_ptr<ReferenceLine> ptr_ref_line_;
  std::shared_ptr<STGraph> ptr_st_graph_;
  std::vector<std::vector<common::Box2d>> predicted_obstacle_box_;
  common::ThreadPool *thread_pool_ = nullptr;
  vehicle_state::VehicleParams vehicle_params_{};
  double lon_buffer_{};
  double lat_buffer_{};
  double lookahead_time_{};
  double delta_t_{};

};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_LOCAL_PLANNER_INCLUDE_COLLISION_CHECKER_COLLISION_CHECKER_HPP_
