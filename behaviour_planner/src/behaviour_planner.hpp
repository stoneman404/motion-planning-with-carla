
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNER_INCLUDE_BEHAVIOUR_PLANNER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNER_INCLUDE_BEHAVIOUR_PLANNER_HPP_
#include <string>
#include <unordered_map>
#include <memory>
#include <planning_msgs/TrajectoryPoint.h>
#include <planning_msgs/Trajectory.h>
#include <thread_pool/thread_pool.hpp>
#include <tf/transform_datatypes.h>
#include "behaviour_strategy/behaviour_strategy.hpp"
#include "agent/agent.hpp"
#include <carla_msgs/CarlaTrafficLightStatusList.h>
#include <carla_msgs/CarlaTrafficLightInfo.h>
#include <behaviour_strategy/mpdm_planner/mpdm_planner.hpp>
#include "planning_msgs/Behaviour.h"

namespace planning {

class BehaviourPlanner {
 public:
  BehaviourPlanner() = default;
  ~BehaviourPlanner() = default;
  explicit BehaviourPlanner(const ros::NodeHandle &nh);
  void RunOnce();
 private:
  bool GetKeyAgents();
  void VisualizeBehaviourTrajectories(const Behaviour &behaviour);
  static bool ConvertBehaviourToRosMsg(const Behaviour &behaviour, planning_msgs::Behaviour &behaviour_msg);

 private:
  ros::NodeHandle nh_;
  int pool_size_ = 6;
  double sample_key_agent_lat_threshold_;
  double sample_min_lon_threshold_;
  std::string planner_type_;
  int ego_vehicle_id_ = -1;
  bool has_ego_vehicle_ = false;
  PolicySimulateConfig simulate_config_;

  ////////////////// ServiceClinet //////////////////////
  ros::ServiceClient get_waypoint_client_;

  ////////////////////// Subscriber /////////////////////
  ros::Subscriber ego_vehicle_subscriber_;
  ros::Subscriber objects_subscriber_;
  ros::Subscriber ego_vehicle_info_subscriber_;

  /////////////////////Publisher///////////////////////
  ros::Publisher behaviour_publisher_;
  ros::Publisher visualized_behaviour_trajectories_publisher_;

  carla_msgs::CarlaEgoVehicleInfo ego_vehicle_info_;
  carla_msgs::CarlaEgoVehicleStatus ego_vehicle_status_;
  carla_msgs::CarlaTrafficLightStatusList traffic_light_status_list_;
  std::unordered_map<int, carla_msgs::CarlaTrafficLightInfo> traffic_lights_info_list_;
//  std::unordered_map<int, derived_object_msgs::Object> objects_map_;
  std::unordered_map<int, Agent> agent_set_;
  std::unordered_map<int, Agent> key_agent_set_;
  std::unique_ptr<BehaviourStrategy> behaviour_strategy_;

  std::unique_ptr<common::ThreadPool> thread_pool_;
};
}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_BEHAVIOUR_PLANNER_INCLUDE_BEHAVIOUR_PLANNER_HPP_
