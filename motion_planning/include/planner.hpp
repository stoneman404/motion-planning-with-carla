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
#include "vehicle_state/vehicle_state.hpp"

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

  /**
   *
   */
  void RunOnce();

  /**
   *
   * @param ego_vehicle_info
   * @param vehicle_status
   * @param trajectory
   * @return
   */
  bool Plan(
      const carla_msgs::CarlaEgoVehicleInfo &ego_vehicle_info,
      const carla_msgs::CarlaEgoVehicleStatus &vehicle_status,
      planning_msgs::Trajectory::Ptr trajectory);

 protected:

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

 private:
  bool has_init_vehicle_params_ = false;
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

};

}

#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_HPP_
