#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_HPP_
#include <ros/ros.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <derived_object_msgs/ObjectArray.h>
#include <visualization_msgs/Marker.h>
#include <carla_msgs/CarlaActorList.h>
#include <carla_msgs/CarlaTrafficLightStatusList.h>
#include <carla_msgs/CarlaTrafficLightInfoList.h>

#include <planning_msgs/Trajectory.h>
#include <carla_waypoint_types/GetActorWaypoint.h>
#include <carla_waypoint_types/GetWaypoint.h>
#include <planning_srvs/Route.h>
#include <unordered_map>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "maneuver_planner/maneuver_planner.hpp"
#include "vehicle_state/vehicle_state.hpp"
#include "thread_pool.hpp"

namespace planning {

class Planner {
 public:
  Planner() = default;

  /**
   *
   * @param nh
   */
  explicit Planner(const ros::NodeHandle &nh);

  /**
   *
   */
  ~Planner() = default;

  void Launch();

  /**
   *
   */
  void RunOnce();

 private:

  /**
   *
   */
  void InitPublisher();

  /**
   *
   */
  void InitSubscriber();

  /**
   *
   */
  void InitServiceClient();

  /**
   *
   * @return
   */
  bool UpdateVehicleStatus();

  /**
   *
   * @return
   */
  bool UpdateObstacleStatus();

  /**
   *
   * @return
   */
  bool UpdateTrafficLights();

  /**
   *
   * @param object_id
   * @param carla_waypoint
   * @return
   */
  bool GetWayPoint(const int &object_id,
                   carla_waypoint_types::CarlaWaypoint &carla_waypoint);
  /**
   *
   * @return
   */
  std::vector<planning_msgs::TrajectoryPoint> GetStitchingTrajectory(const ros::Time &current_time_stamp,
                                                                     double planning_cycle_time,
                                                                     size_t preserve_points_num);
  void VisualizeValidTrajectories(const std::vector<planning_msgs::Trajectory> &valid_trajectories) const;
  void VisualizeOptimalTrajectory(const planning_msgs::Trajectory &optimal_trajectory) const;
  void VisualizeTrafficLightBox();
  void VisualizeReferenceLine(std::vector<std::shared_ptr<ReferenceLine>> &ref_lines);

 private:
  bool has_maneuver_planner_ = false;
  bool has_init_vehicle_params_ = false;
  bool has_history_trajectory_ = false;
  int ego_vehicle_id_ = -1;
  nav_msgs::Odometry ego_odometry_;
  carla_msgs::CarlaEgoVehicleInfo ego_vehicle_info_;
  carla_msgs::CarlaEgoVehicleStatus ego_vehicle_status_;
  carla_msgs::CarlaTrafficLightStatusList traffic_light_status_list_;
  std::unordered_map<int, carla_msgs::CarlaTrafficLightInfo> traffic_lights_info_list_;
  std::unordered_map<int, derived_object_msgs::Object> objects_map_;
  derived_object_msgs::Object ego_object_;
  ros::NodeHandle nh_;
  geometry_msgs::PoseStamped goal_pose_;
  geometry_msgs::PoseWithCovarianceStamped init_pose_;
  planning_msgs::Trajectory history_trajectory_;

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
  ros::Subscriber goal_pose_subscriber_;
  ros::Subscriber init_pose_subscriber_;

  /////////////////////// Publisher /////////////////////
  ros::Publisher trajectory_publisher_;
  ros::Publisher visualized_trajectory_publisher_;
  ros::Publisher visualized_valid_trajectories_publisher_;
  ros::Publisher visualized_reference_lines_publisher_;
  ros::Publisher visualized_traffic_light_box_publisher_;

  //////////////////////maneuver planner///////////////
  std::unique_ptr<ManeuverPlanner> maneuver_planner_;

  /////////////////// thread pool///////////////////
  size_t thread_pool_size_ = 8;
  std::unique_ptr<ThreadPool> thread_pool_;

};
}

#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_HPP_
