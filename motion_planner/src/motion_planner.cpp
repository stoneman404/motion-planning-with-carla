#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <planning_msgs/Behaviour.h>
#include "motion_planner.hpp"
#include "name/string_name.hpp"
#include "planning_config.hpp"

namespace planning {
MotionPlanner::MotionPlanner(const ros::NodeHandle &nh) : nh_(nh), thread_pool_size_(8) {
  PlanningConfig::Instance().UpdateParams(nh_);
  this->thread_pool_ = std::make_unique<common::ThreadPool>(thread_pool_size_);
  this->vehicle_state_ = std::make_unique<vehicle_state::VehicleState>();
  this->InitPublisher();
  this->InitSubscriber();
  this->InitServiceClient();
}

void MotionPlanner::Launch() {

  ros::Rate loop_rate(PlanningConfig::Instance().loop_rate());
  while (ros::ok()) {
    std::cout << "loop rate: " << PlanningConfig::Instance().loop_rate() << std::endl;
    this->RunOnce();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void MotionPlanner::RunOnce() {
  ros::Time current_time_stamp = ros::Time::now();
  if (ego_vehicle_id_ == -1) {
    return;
  }
  if (objects_map_.find(ego_vehicle_id_) == objects_map_.end()) {
    ROS_FATAL("[MotionPlanner::RunOnce], no ego vehicle founded");
    return;
  }
  ego_object_ = objects_map_[ego_vehicle_id_];
  vehicle_state_->Update(ego_vehicle_status_, ego_vehicle_info_, ego_object_);
  if (!UpdateSurroudingObstacles()) {
    ROS_FATAL("UpdateSurroudingObstacles failed");
    return;
  }

//  if (!UpdateTrafficLights()) {
//    ROS_DEBUG("UpdateTrafficLights failed");
//    return;
//  }
//  ros::Time end = ros::Time::now();
//  auto maneuver_planner_start_time_stamp = ros::Time::now();
//  ROS_WARN("RunOnce: update vehicle, traffic light, obstacle elapsed %lf ms",
//           static_cast<double>((end - current_time_stamp).toNSec()) / 1000000.0);
//  VisualizeTrafficLightBox();
//  if (!has_maneuver_planner_) {
//    maneuver_planner_ = std::make_unique<ManeuverPlanner>(nh_, thread_pool_.get());
//    has_maneuver_planner_ = true;
//  }
////  auto init_trajectory_point = MotionPlanner::GetStitchingTrajectory();
//  planning_msgs::TrajectoryPoint init_trajectory_point;
//  init_trajectory_point.path_point.x = VehicleState::Instance().GetKinoDynamicVehicleState().x;
//  init_trajectory_point.path_point.y = VehicleState::Instance().GetKinoDynamicVehicleState().y;
//  init_trajectory_point.path_point.theta = VehicleState::Instance().GetKinoDynamicVehicleState().theta;
//  auto maneuver_status = maneuver_planner_->Process(init_trajectory_point);
//  end = ros::Time::now();
//  ROS_WARN("RunOnce: maneuver_planner_ Process Elapsed %lf ms",
//           static_cast<double>((end - maneuver_planner_start_time_stamp).toNSec()) / 1000000.0);
//  this->VisualizeReferenceLine(maneuver_planner_->multable_ref_line());
//  this->VisualizeRoute(maneuver_planner_->multable_routes());
//  if (maneuver_status != ManeuverStatus::kSuccess) {
//    ROS_FATAL("ManeuverPlanner failed, [maneuver_status: %ud]", static_cast<uint32_t>(maneuver_status));
//  } else {
//    ROS_INFO("ManeuverPlanner success, [maneuver_status: %ud]", static_cast<uint32_t>(maneuver_status));
//  }
//  auto optimal_trajectory = maneuver_planner_->optimal_trajectory();
////  for (const auto &tp : optimal_trajectory.trajectory_points) {
////    std::cout << "x : " << tp.path_point.x << " y: " << tp.path_point.y << " vel : " << tp.vel << " acc : " << tp.acc
////              << std::endl;
////  }
//  optimal_trajectory.header.stamp = current_time_stamp;
//  this->VisualizeOptimalTrajectory(optimal_trajectory);
//  history_trajectory_ = optimal_trajectory;
//  trajectory_publisher_.publish(optimal_trajectory);
//  auto valid_trajectories = maneuver_planner_->valid_trajectories();
//  if (!valid_trajectories.empty()) {
//    this->VisualizeValidTrajectories(valid_trajectories);
//  }
//  end = ros::Time::now();
//  ROS_WARN("[MotionPlanner], RunOnce Time: %lf ms",
//           static_cast<double>((end - current_time_stamp).toNSec()) / 1000000.0);
}

void MotionPlanner::InitPublisher() {
  this->trajectory_publisher_ = nh_.advertise<planning_msgs::Trajectory>(
      common::topic::kPublishedTrajectoryName, 1);
  this->visualized_trajectory_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      common::topic::kVisualizedTrajectoryName, 1);
//  this->visualized_trajectory_publisher_ = nh_.advertise<nav_msgs::Path>(topic::kVisualizedTrajectoryName, 10);
  this->visualized_valid_trajectories_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      common::topic::kVisualizedValidTrajectoriesName, 1);
  this->visualized_reference_lines_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      common::topic::kVisualizedReferenceLinesName, 1);
  this->visualized_traffic_light_box_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      common::topic::kVisualizedTrafficLightBoxName, 1);
  this->visualized_obstacle_trajectory_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      common::topic::kVisualizedObstacleTrajectoriesName, 1);
}

void MotionPlanner::InitSubscriber() {

  this->behaviour_subscriber_ = nh_.subscribe<planning_msgs::Behaviour>(
      common::topic::kBehaviourName, 10,
      [this](const planning_msgs::Behaviour::ConstPtr &behaviour) {
        this->behaviour_ = *behaviour;
      });
  this->ego_vehicle_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
      common::topic::kEgoVehicleStatusName, 1,
      [this](const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &ego_vehicle_status) {
        this->ego_vehicle_status_ = *ego_vehicle_status;
      });
  this->traffic_lights_subscriber_ = nh_.subscribe<carla_msgs::CarlaTrafficLightStatusList>(
      common::topic::kTrafficLigthsName, 1,
      [this](const carla_msgs::CarlaTrafficLightStatusList::ConstPtr &traffic_light_status_list) {
        this->traffic_light_status_list_ = *traffic_light_status_list;
        ROS_INFO("the traffic light status list size: %zu",
                 traffic_light_status_list_.traffic_lights.size());
      });
  this->traffic_lights_info_subscriber_ = nh_.subscribe<carla_msgs::CarlaTrafficLightInfoList>(
      common::topic::kTrafficLightsInfoName,
      1,
      [this](const carla_msgs::CarlaTrafficLightInfoList::ConstPtr &traffic_lights_info_list) {
        traffic_lights_info_list_ = decltype(traffic_lights_info_list_)();
        for (const auto &traffic_light_info : traffic_lights_info_list->traffic_lights) {
          traffic_lights_info_list_.emplace(traffic_light_info.id, traffic_light_info);
        }
      });
  this->ego_vehicle_info_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleInfo>(
      common::topic::kEgoVehicleInfoName, 10,
      [this](const carla_msgs::CarlaEgoVehicleInfo::ConstPtr &ego_vehicle_info) {
        ego_vehicle_info_ = *ego_vehicle_info;
        this->ego_vehicle_id_ = ego_vehicle_info_.id;
        ROS_INFO("the ego_vehicle_id_: %i", ego_vehicle_id_);
      });
  this->objects_subscriber_ = nh_.subscribe<derived_object_msgs::ObjectArray>(
      common::topic::kObjectsName, 10,
      [this](const derived_object_msgs::ObjectArray::ConstPtr &object_array) {
        this->objects_map_.clear();
        for (const auto &object : object_array->objects) {
          objects_map_.emplace(object.id, object);
        }
        ROS_INFO("the objects map_ size is: %lu", objects_map_.size());
      });
}

void MotionPlanner::InitServiceClient() {
  this->get_waypoint_client_ = nh_.serviceClient<carla_waypoint_types::GetWaypoint>(
      common::service::kGetEgoWaypontServiceName);
  this->get_actor_waypoint_client_ = nh_.serviceClient<carla_waypoint_types::GetActorWaypoint>(
      common::service::kGetActorWaypointServiceName);
  ROS_DEBUG("MotionPlanner::InitServiceClient finished");
}

bool MotionPlanner::UpdateSurroudingObstacles() {
  obstacles_.clear();
  if (traffic_lights_info_list_.size() != traffic_light_status_list_.traffic_lights.size()) {
    return false;
  }
  constexpr double kMaxRadius = 100.0;
  for (const auto &object : objects_map_) {
    double distance = std::hypot(ego_object_.pose.position.x - object.second.pose.position.x,
                                 ego_object_.pose.position.y - object.second.pose.position.y);
    if (distance > kMaxRadius) {
      continue;
    }
    if (object.first == ego_object_.id) {
      continue;
    }
    obstacles_.emplace_back(std::make_shared<Obstacle>(object.second));
  }
  for (size_t i = 0; i < traffic_light_status_list_.traffic_lights.size(); ++i) {
    auto obstacle = std::make_shared<Obstacle>(traffic_lights_info_list_[i],
                                               traffic_light_status_list_.traffic_lights[i]);
    double distance = std::hypot(obstacle->x() - ego_object_.pose.position.x,
                                 obstacle->y() - ego_object_.pose.position.y);
    if (distance > kMaxRadius) {
      continue;
    }
    obstacles_.emplace_back(obstacle);
  }
  ROS_INFO("the obstacle size is %zu", obstacles_.size());
  return true;
}

bool MotionPlanner::GetWayPoint(const int &object_id,
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

void MotionPlanner::VisualizeValidTrajectories(const std::vector<planning_msgs::Trajectory> &valid_trajectories) const {
  visualization_msgs::MarkerArray valid_trajectories_marker_array;
  ROS_INFO("MotionPlanner  valid trajectories size %zu", valid_trajectories.size());
  int i = 1;
  for (const auto &valid_trajectory : valid_trajectories) {
    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.id = i;
    trajectory_marker.scale.x = 0.1;
    trajectory_marker.color.a = 1.0;
    trajectory_marker.color.r = 1.0;
    trajectory_marker.pose.orientation.w = 1.0;
    trajectory_marker.header.stamp = ros::Time::now();
    trajectory_marker.header.frame_id = "map";
    trajectory_marker.action = visualization_msgs::Marker::ADD;
    for (const auto &tp : valid_trajectory.trajectory_points) {
      geometry_msgs::Point p;
      p.x = tp.path_point.x;
      p.y = tp.path_point.y;
      p.z = 2;
      trajectory_marker.points.push_back(p);
    }
    i++;
    valid_trajectories_marker_array.markers.push_back(trajectory_marker);
  }
  visualized_valid_trajectories_publisher_.publish(valid_trajectories_marker_array);
}

void MotionPlanner::VisualizeOptimalTrajectory(const planning_msgs::Trajectory &optimal_trajectory) const {
  visualization_msgs::Marker optimal_trajectory_marker;
  optimal_trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
  optimal_trajectory_marker.header.stamp = ros::Time::now();
  optimal_trajectory_marker.header.frame_id = "map";
  optimal_trajectory_marker.action = visualization_msgs::Marker::ADD;
  optimal_trajectory_marker.color.a = 1.0;
  optimal_trajectory_marker.color.b = 0.8;
  optimal_trajectory_marker.color.r = 1.0;
//  optimal_trajectory_marker.color.g =0.7;
  optimal_trajectory_marker.scale.x = 0.2;
  optimal_trajectory_marker.pose.orientation.w = 1.0;
  optimal_trajectory_marker.id = 0;

  for (const auto &tp : optimal_trajectory.trajectory_points) {
    geometry_msgs::Point p;
    p.x = tp.path_point.x;
    p.y = tp.path_point.y;
    p.z = 2;
    optimal_trajectory_marker.points.push_back(p);
  }
  visualized_trajectory_publisher_.publish(optimal_trajectory_marker);

}

std::vector<planning_msgs::TrajectoryPoint> MotionPlanner::GetStitchingTrajectory(const ros::Time &current_time_stamp,
                                                                                  double planning_cycle_time,
                                                                                  size_t preserve_points_num) {
  return std::vector<planning_msgs::TrajectoryPoint>();

}

void MotionPlanner::VisualizeTrafficLightBox() {
  visualization_msgs::MarkerArray traffic_light_boxes_markers;
  for (const auto &traffic_light : traffic_lights_info_list_) {
    visualization_msgs::Marker traffic_light_marker;
    traffic_light_marker.type = visualization_msgs::Marker::CUBE;
    traffic_light_marker.header.stamp = ros::Time::now();
    traffic_light_marker.action = visualization_msgs::Marker::ADD;
    traffic_light_marker.header.frame_id = "map";
    traffic_light_marker.color.a = 1.0;
    traffic_light_marker.color.r = 0.5;
    traffic_light_marker.color.g = 0.2;
    traffic_light_marker.color.b = 0.6;
    traffic_light_marker.scale = traffic_light.second.trigger_volume.size;
    traffic_light_marker.pose.position = traffic_light.second.transform.position;
    traffic_light_marker.pose.orientation = traffic_light.second.transform.orientation;
    traffic_light_marker.id = traffic_light.first;
    traffic_light_boxes_markers.markers.push_back(traffic_light_marker);
  }
  visualized_traffic_light_box_publisher_.publish(traffic_light_boxes_markers);
}

void MotionPlanner::VisualizeReferenceLine(std::vector<std::shared_ptr<ReferenceLine>> &ref_lines) {
  visualization_msgs::MarkerArray marker_array;
  int i = 100;
  for (const auto &ref_line : ref_lines) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.id = i;
    marker.scale.x = 0.1;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    const double ds = 0.5;
    double s = 0.0;
    while (s <= ref_line->Length()) {
      auto ref_point = ref_line->GetReferencePoint(s);
      geometry_msgs::Point pt;
      pt.x = ref_point.x();
      pt.y = ref_point.y();
      pt.z = 2;
      marker.points.push_back(pt);
      s += ds;
    }
    marker_array.markers.push_back(marker);
    i++;
  }
  visualized_reference_lines_publisher_.publish(marker_array);
}

}