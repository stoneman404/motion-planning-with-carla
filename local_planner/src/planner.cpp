#include <geometry_msgs/PoseStamped.h>
#include "planner.hpp"
#include "string_name.hpp"
#include "planning_config.hpp"
#include "vehicle_state/vehicle_state.hpp"
#include "obstacle_filter/obstacle.hpp"
#include "obstacle_filter/obstacle_filter.hpp"
#include "planning_context.hpp"

namespace planning {
Planner::Planner(const ros::NodeHandle &nh) : nh_(nh) {
  PlanningConfig::Instance().UpdateParams(nh_);
  this->InitPublisher();
  this->InitSubscriber();
  this->InitServiceClient();
}

void Planner::RunOnce() {
  if (ego_vehicle_id_ == -1) {
    return;
  }
  /// initialize the vehicle params
  if (!has_init_vehicle_params_) {
    PlanningConfig::Instance().UpdateVehicleParams(ego_object_, ego_vehicle_info_);
    has_init_vehicle_params_ = true;
  }
}

void Planner::InitPublisher() {
  this->trajectory_publisher_ = nh_.advertise<planning_msgs::Trajectory>(
      topic::kPublishedTrajectoryName, 10);
  this->visualized_trajectory_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      topic::kVisualizedTrajectoryName, 10);
}

void Planner::InitSubscriber() {

  this->ego_vehicle_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
      topic::kEgoVehicleStatusName, 10,
      [this](const carla_msgs::CarlaEgoVehicleStatus::ConstPtr ego_vehicle_status) {
        this->ego_vehicle_status_ = *ego_vehicle_status;
      });

  this->traffic_lights_subscriber_ = nh_.subscribe<carla_msgs::CarlaTrafficLightStatusList>(
      topic::kTrafficLigthsName, 10,
      [this](const carla_msgs::CarlaTrafficLightStatusList::ConstPtr traffic_light_status_list) {
        this->traffic_light_status_list_ = *traffic_light_status_list;
        ROS_INFO("the traffic ligth status list size: %zu",
                 traffic_light_status_list_.traffic_lights.size());
      });

  this->ego_vehicle_info_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleInfo>(
      topic::kEgoVehicleInfoName, 10,
      [this](const carla_msgs::CarlaEgoVehicleInfo::ConstPtr ego_vehicle_info) {
        ego_vehicle_info_ = *ego_vehicle_info;
        this->ego_vehicle_id_ = ego_vehicle_info_.id;
        ROS_INFO("the ego_vehicle_id_: %i", ego_vehicle_id_);
      });

  this->objects_subscriber_ = nh_.subscribe<derived_object_msgs::ObjectArray>(
      topic::kObjectsName, 10,
      [this](const derived_object_msgs::ObjectArray::ConstPtr object_array) {
        this->objects_map_.clear();
        for (const auto &object : object_array->objects) {
          objects_map_.emplace(object.id, object);
        }
        if (ego_vehicle_id_ != -1) {
          ego_object_ = objects_map_[ego_vehicle_id_];
        }
        ROS_INFO("the objects map_ size is: %lu", objects_map_.size());
      });

  this->ego_vehicle_odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(
      topic::kEgoVehicleOdometryName, 10,
      [this](const nav_msgs::Odometry::ConstPtr ego_odometry) {
        this->ego_odometry_ = *ego_odometry;

      });

  this->init_pose_subscriber_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      topic::kInitialPoseName, 10,
      [this](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr init_pose) {
        this->init_pose_ = *init_pose;
        PlanningContext::Instance().UpdateGlobalInitPose(init_pose_);
        ROS_INFO("the init_pose_: x: %lf, y: %lf",
                 init_pose_.pose.pose.position.x, init_pose_.pose.pose.position.y);
      });

  this->goal_pose_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      topic::kGoalPoseName, 10,
      [this](const geometry_msgs::PoseStamped::ConstPtr goal_pose) {
        this->goal_pose_ = *goal_pose;
        PlanningContext::Instance().UpdateGlobalGoalPose(goal_pose_);
        ROS_INFO("the goal_pose_ : x: %lf, y: %lf",
                 goal_pose_.pose.position.x, goal_pose_.pose.position.y);
      });
}

void Planner::InitServiceClient() {
  this->get_waypoint_client_ = nh_.serviceClient<carla_waypoint_types::GetWaypoint>(
      service::kGetEgoWaypontServiceName);
  this->get_actor_waypoint_client_ = nh_.serviceClient<carla_waypoint_types::GetActorWaypoint>(
      service::kGetActorWaypointServiceName);
//  this->route_service_client_ = nh_.serviceClient<planning_srvs::Route>(service::kRouteServiceName);

}
//
//bool Planner::ReRoute(const geometry_msgs::Pose &start,
//                      const geometry_msgs::Pose &destination,
//                      planning_srvs::RouteResponse &response) {
//  planning_srvs::Route srv;
//  srv.request.start_pose = start;
//  srv.request.end_pose = destination;
//  if (!route_service_client_.call(srv)) {
//    ROS_FATAL("[Planner::Reroute], Failed to ReRoute!");
//    return false;
//  } else {
//    response = srv.response;
//    ROS_INFO("[Planner::Reroute], Reroute SUCCESSFUL,"
//             " the route size is %zu", response.route.size());
//    return true;
//  }
//}

// main loop function
bool Planner::Plan(const carla_msgs::CarlaEgoVehicleInfo &ego_vehicle_info,
                   const carla_msgs::CarlaEgoVehicleStatus &vehicle_status,
                   planning_msgs::Trajectory::Ptr trajectory) {
  if (trajectory == nullptr) {
    ROS_FATAL("[Planner::Plan], the trajectory ptr is NULL");
    return false;
  }

}

bool Planner::UpdateObstacleStatus() {
  /// update the obstacle at each run
  ObstacleFilter::Instance().UpdateObstacles(objects_map_, ego_odometry_);
  auto &obstacles = ObstacleFilter::Instance().multable_obstacles();
  int failed_count = 0;

  for (auto &obstacle : obstacles) {
    carla_waypoint_types::CarlaWaypoint way_point;
    if (!this->GetWayPoint(obstacle->Id(), way_point)) {
      ROS_WARN("the obstacle[id: %i] failed to get way_point.", obstacle->Id());
      failed_count++;
      continue;
    }

    obstacle->set_road_id(way_point.road_id);
    obstacle->set_section_id(way_point.section_id);
    obstacle->set_lane_id(way_point.lane_id);
  }

  // if all obstacles are failed to get way_point, something going wrong
  return failed_count < ObstacleFilter::Instance().ObstaclesSize();
}

bool Planner::UpdateVehicleStatus() {
  /// update the vehicle state at each run
  VehicleState::Instance().Update(ego_vehicle_status_, ego_odometry_, ego_vehicle_info_);
  auto ego_id = ego_vehicle_info_.id;
  carla_waypoint_types::CarlaWaypoint carla_way_point;

  if (!this->GetWayPoint(ego_id, carla_way_point)) {
    return false;
  }

  VehicleState::Instance().set_lane_id(carla_way_point.lane_id);
  VehicleState::Instance().set_section_id(carla_way_point.section_id);
  VehicleState::Instance().set_road_id(carla_way_point.road_id);
}

bool Planner::GetWayPoint(const int &object_id,
                          carla_waypoint_types::CarlaWaypoint &carla_waypoint) {

  if (object_id < 0) {
    return false;
  }

  carla_waypoint_types::GetActorWaypoint srv;
  srv.request.id = object_id;
  if (!get_actor_waypoint_client_.call(srv)) {
    ROS_FATAL("Cannot UpdateVehicleStatus");
    return false;
  }

  carla_waypoint = srv.response.waypoint;
  return true;
}

}