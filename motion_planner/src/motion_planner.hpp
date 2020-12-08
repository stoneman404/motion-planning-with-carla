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

namespace planning {

class MotionPlanner {
 public:
  MotionPlanner() = default;
  explicit MotionPlanner(const ros::NodeHandle &nh);
  ~MotionPlanner() = default;
  void Launch();

 private:
  void RunOnce();
  void InitPublisher();
  void InitSubscriber();
  void InitServiceClient();
  static void GenerateEmergencyStopTrajectory(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                              planning_msgs::Trajectory &emergency_stop_trajectory);
  bool GetPlanningTargetFromBehaviour(const planning_msgs::Behaviour &behaviour,
                                      std::vector<PlanningTarget> &planning_targets);
  std::vector<planning_msgs::TrajectoryPoint> GetStitchingTrajectory(const ros::Time &current_time_stamp,
                                                                     double planning_cycle_time,
                                                                     size_t preserve_points_num);

  static planning_msgs::TrajectoryPoint ComputeTrajectoryPointFromVehicleState(double planning_cycle_time,
                                                                               const vehicle_state::KinoDynamicState &kinodynamic_state);

  static std::vector<planning_msgs::TrajectoryPoint> ComputeReinitStitchingTrajectory(double planning_cycle_time,
                                                                                      const vehicle_state::KinoDynamicState &kino_dynamic_state);

  static size_t GetTimeMatchIndex(double relative,
                                  double eps,
                                  const std::vector<planning_msgs::TrajectoryPoint> &trajectory);

  static size_t GetPositionMatchedIndex(const std::pair<double, double> &xy,
                                        const std::vector<planning_msgs::TrajectoryPoint> &trajectory);

  /**
   * @brief get lateral and longitudinal distance from reference path point.
   * @param x:
   * @param y:
   * @param point: the ref point
   * @return pair.first: longitudinal distance + refpoint.s , pair.second: lateral distance
   */
  static std::pair<double, double> GetLatAndLonDistFromRefPoint(double x,
                                                                double y,
                                                                const planning_msgs::PathPoint &point);

  void VisualizeValidTrajectories(const std::vector<planning_msgs::Trajectory> &valid_trajectories) const;
  void VisualizeOptimalTrajectory(const planning_msgs::Trajectory &optimal_trajectory) const;
  void VisualizeTrafficLightBox();
  void VisualizeReferenceLine(std::vector<std::shared_ptr<ReferenceLine>> &ref_lines);
  static bool GetLocalGoal(PlanningTarget &planning_target);

 private:
  bool has_history_trajectory_ = false;
  int ego_vehicle_id_ = -1;
  std::unique_ptr<vehicle_state::VehicleState> vehicle_state_;
  std::vector<std::shared_ptr<Obstacle>> obstacles_;
  planning_msgs::Behaviour behaviour_;
  carla_msgs::CarlaEgoVehicleInfo ego_vehicle_info_;
  carla_msgs::CarlaEgoVehicleStatus ego_vehicle_status_;
//  carla_msgs::CarlaTrafficLightStatusList traffic_light_status_list_;
  std::unordered_map<int, carla_msgs::CarlaTrafficLightStatus> traffic_light_status_list_;
  std::unordered_map<int, carla_msgs::CarlaTrafficLightInfo> traffic_lights_info_list_;
  std::unordered_map<int, derived_object_msgs::Object> objects_map_;
  derived_object_msgs::Object ego_object_;
  ros::NodeHandle nh_;
  planning_msgs::Trajectory history_trajectory_;
  std::unique_ptr<TrajectoryPlanner> trajectory_planner_;

  ////////////////// ServiceClinet //////////////////////
  ros::ServiceClient get_actor_waypoint_client_;
  ros::ServiceClient get_waypoint_client_;

  ////////////////////// Subscriber /////////////////////
  ros::Subscriber ego_vehicle_subscriber_;
  ros::Subscriber objects_subscriber_;
  ros::Subscriber traffic_lights_subscriber_;
  ros::Subscriber traffic_lights_info_subscriber_;
  ros::Subscriber ego_vehicle_info_subscriber_;
  ros::Subscriber ego_vehicle_odometry_subscriber_;
  ros::Subscriber behaviour_subscriber_;
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

};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_INCLUDE_PLANNER_HPP_
