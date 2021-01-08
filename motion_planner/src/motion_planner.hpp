#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_INCLUDE_PLANNER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_INCLUDE_PLANNER_HPP_
#include <ros/ros.h>
#include <unordered_map>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <derived_object_msgs/ObjectArray.h>

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaActorList.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <carla_msgs/CarlaTrafficLightStatusList.h>
#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include <carla_waypoint_types/GetActorWaypoint.h>
#include <carla_waypoint_types/GetWaypoint.h>

#include "vehicle_state/vehicle_state.hpp"
#include "thread_pool/thread_pool.hpp"
#include "obstacle_manager/obstacle.hpp"
#include <planning_msgs/Trajectory.h>
#include <planning_msgs/Behaviour.h>
#include <reference_line/reference_line.hpp>
#include "trajectory_planner.hpp"
#include "frenet_lattice_planner/frenet_lattice_planner.hpp"
#include "behaviour_planner/behaviour_strategy/behaviour_strategy.hpp"
#include "behaviour_planner/behaviour_strategy/mpdm_planner/mpdm_planner.hpp"

namespace planning {

class MotionPlanner {
 public:
  MotionPlanner() = default;
  explicit MotionPlanner(const ros::NodeHandle &nh);
  ~MotionPlanner();
  void Launch();

 private:
  void RunOnce();
  void InitPublisher();
  void InitSubscriber();
  void InitServiceClient();

  /**
   * @brief: make behaviour agent set for behaviour planning
   * @return: the behaviour agent set, std::unordered_map<int, Agent>
   */
  std::unordered_map<int, Agent> MakeBehaviourAgentSet();

  /**
   * @brief: get key agent set from behaviour agent set and ego agent state
   * @param agent_set
   * @param ego_id
   * @return
   */
  std::unordered_map<int, Agent> GetKeyAgents(
      const std::unordered_map<int, Agent> &agent_set, int ego_id);

  /**
   * @brief: predict agent behaviour.
   * @param key_agent_set
   * @param ego_id
   * @return
   */
  bool PredictAgentsBehaviours(
      std::unordered_map<int, Agent> &key_agent_set, int ego_id);

  /**
   * @brief: add agent potential reference line
   * @param state
   * @param lane
   * @param lookahead_length
   * @param lookback_length
   * @param smooth
   * @param ptr_potential_lanes
   * @return
   */
  static bool AddAgentPotentialReferenceLines(
      const vehicle_state::KinoDynamicState &state,
      const planning_msgs::Lane &lane,
      double lookahead_length,
      double lookback_length,
      bool smooth,
      std::vector<ReferenceLine> *ptr_potential_lanes);

  /**
   * @brief: get agent potential reference lines
   * @param agent_state
   * @param agent_id
   * @param lookahead_length
   * @param lookback_length
   * @param potential_reference_lines
   * @return
   */
  bool GetAgentPotentialRefLanes(const vehicle_state::KinoDynamicState &agent_state,
                                 int agent_id,
                                 double lookahead_length,
                                 double lookback_length,
                                 std::vector<ReferenceLine> *potential_reference_lines);

  /**
   * @brief: get ego vehicle routes
   * @param start_pose
   * @param destination
   * @return
   */
  bool GetEgoVehicleRoutes(geometry_msgs::Pose &start_pose, geometry_msgs::Pose &destination);

  /**
   * @brief: generate emergency stop trajectory
   * @param init_trajectory_point
   * @param emergency_stop_trajectory
   */
  static void GenerateEmergencyStopTrajectory(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                              planning_msgs::Trajectory &emergency_stop_trajectory);

  /**
   * @brief: get planning target from behaviour
   * @param behaviour
   * @param planning_targets
   * @return
   */
  bool GetPlanningTargetFromBehaviour(const Behaviour &behaviour,
                                      std::vector<PlanningTarget> &planning_targets);

  /**
   * @brief: stitching history trajectory and planned trajectory
   * @param current_time_stamp
   * @param planning_cycle_time
   * @param preserve_points_num
   * @return
   */
  std::vector<planning_msgs::TrajectoryPoint> GetStitchingTrajectory(const ros::Time &current_time_stamp,
                                                                     double planning_cycle_time,
                                                                     size_t preserve_points_num);

  /**
   * @brief: compute the trajectory point from vehicle state
   * @param planning_cycle_time
   * @param kinodynamic_state
   * @return
   */
  static planning_msgs::TrajectoryPoint ComputeTrajectoryPointFromVehicleState(
      double planning_cycle_time,
      const vehicle_state::KinoDynamicState &kinodynamic_state);

  /**
   * @brief: compute reinit stitchinig trajectory
   * @param planning_cycle_time
   * @param kino_dynamic_state
   * @return
   */
  static std::vector<planning_msgs::TrajectoryPoint> ComputeReinitStitchingTrajectory(
      double planning_cycle_time,
      const vehicle_state::KinoDynamicState &kino_dynamic_state);

  /**
   * @brief: get matched index from time
   * @param relative
   * @param eps
   * @param trajectory
   * @return
   */
  static size_t GetTimeMatchIndex(
      double relative,
      double eps,
      const std::vector<planning_msgs::TrajectoryPoint> &trajectory);

  /**
   * @brief: get the matched index from position
   * @param xy
   * @param trajectory
   * @return
   */
  static size_t GetPositionMatchedIndex(
      const std::pair<double, double> &xy,
      const std::vector<planning_msgs::TrajectoryPoint> &trajectory);
  /**
   * @brief get lateral and longitudinal distance from reference path point.
   * @param x:
   * @param y:
   * @param point: the ref point
   * @return pair.first: longitudinal distance + refpoint.s , pair.second: lateral distance
   */
  static std::pair<double, double> GetLatAndLonDistFromRefPoint(
      double x,
      double y,
      const planning_msgs::PathPoint &point);

  /**
   * @brief: visualize trajectories
   * @param valid_trajectories
   */
  void VisualizeValidTrajectories(const std::vector<planning_msgs::Trajectory> &valid_trajectories) const;

  /**
   * @brief: visualized optimal trajectory
   * @param optimal_trajectory
   */
  void VisualizeOptimalTrajectory(const planning_msgs::Trajectory &optimal_trajectory) const;

  /**
   * @brief: visualize traffic light
   */
  void VisualizeTrafficLightBox();

  /**
   * @brief: visualize reference lines
   * @param ref_lanes
   */
  void VisualizeReferenceLine(std::vector<ReferenceLine> &ref_lanes);

  /**
   * @brief: get local goal
   * @param planning_target
   * @return
   */
  static bool GetLocalGoal(PlanningTarget &planning_target);

 private:
  bool has_history_trajectory_ = false;
  int ego_vehicle_id_ = -1;
  std::unique_ptr<vehicle_state::VehicleState> vehicle_state_;
  std::vector<std::shared_ptr<Obstacle>> obstacles_;
//  planning_msgs::Behaviour behaviour_;
  carla_msgs::CarlaEgoVehicleInfo ego_vehicle_info_;
  carla_msgs::CarlaEgoVehicleStatus ego_vehicle_status_;
  std::unordered_map<int, carla_msgs::CarlaTrafficLightStatus> traffic_light_status_list_;
  std::unordered_map<int, carla_msgs::CarlaTrafficLightInfo> traffic_lights_info_list_;
  std::unordered_map<int, derived_object_msgs::Object> objects_map_;
  derived_object_msgs::Object ego_object_;
  ros::NodeHandle nh_;
  planning_msgs::Trajectory history_trajectory_;
  std::unique_ptr<TrajectoryPlanner> trajectory_planner_;
  std::unique_ptr<BehaviourStrategy> behaviour_planner_;

  ////////////////// ServiceClinet //////////////////////
  ros::ServiceClient get_actor_waypoint_client_;
  ros::ServiceClient get_waypoint_client_;
  ros::ServiceClient get_agent_potential_routes_client_;
  ros::ServiceClient get_ego_vehicle_route_client_;

  ////////////////////// Subscriber /////////////////////
  ros::Subscriber ego_vehicle_subscriber_;
  ros::Subscriber objects_subscriber_;
  ros::Subscriber traffic_lights_subscriber_;
  ros::Subscriber traffic_lights_info_subscriber_;
  ros::Subscriber ego_vehicle_info_subscriber_;
  ros::Subscriber ego_vehicle_odometry_subscriber_;
//  ros::Subscriber behaviour_subscriber_;
  ros::Subscriber goal_pose_subscriber_;
  /////////////////////// Publisher /////////////////////
  ros::Publisher trajectory_publisher_;
  ros::Publisher visualized_trajectory_publisher_;
  ros::Publisher visualized_valid_trajectories_publisher_;
  ros::Publisher visualized_reference_lines_publisher_;
  ros::Publisher visualized_traffic_light_box_publisher_;
  ros::Publisher visualized_obstacle_trajectory_publisher_;

  /////////////////// thread pool///////////////////
  size_t thread_pool_size_ = 6;
  std::unique_ptr<common::ThreadPool> thread_pool_;
  std::unique_ptr<ReferenceInfo> reference_info_;

  std::vector<PlanningTarget> planning_targets_;

};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_INCLUDE_PLANNER_HPP_
