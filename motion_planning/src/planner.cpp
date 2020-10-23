#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include "planner.hpp"
#include "string_name.hpp"
#include "planning_config.hpp"
#include "vehicle_state/vehicle_state.hpp"
#include "obstacle_filter/obstacle.hpp"
#include "obstacle_filter/obstacle_filter.hpp"
#include "planning_context.hpp"
#include "traffic_lights/traffic_light_list.hpp"

namespace planning {
Planner::Planner(const ros::NodeHandle &nh) : nh_(nh), thread_pool_size_(8) {
  PlanningConfig::Instance().UpdateParams(nh_);
  this->thread_pool_ = std::make_unique<ThreadPool>(thread_pool_size_);
  this->InitPublisher();
  this->InitSubscriber();
  this->InitServiceClient();
}

void Planner::Launch() {

  ros::Rate loop_rate(PlanningConfig::Instance().loop_rate());
  while (ros::ok()) {
    std::cout << "loop rate: " << PlanningConfig::Instance().loop_rate() << std::endl;
    this->RunOnce();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Planner::RunOnce() {
  ros::Time current_time_stamp = ros::Time::now();
  if (ego_vehicle_id_ == -1) {
    return;
  }
  /// initialize the vehicle params
  if (!has_init_vehicle_params_) {
    PlanningConfig::Instance().UpdateVehicleParams(ego_object_, ego_vehicle_info_);
    has_init_vehicle_params_ = true;
  }
  if (!UpdateVehicleStatus()) {
    ROS_DEBUG("UpdateVehicleStatus failed");
    return;
  }
  if (!UpdateObstacleStatus()) {
    ROS_DEBUG("UpdateObstacleStatus failed");
    return;
  }
  if (!UpdateTrafficLights()) {
    ROS_DEBUG("UpdateTrafficLights failed");
    return;
  }
  if (!has_maneuver_planner_) {
    maneuver_planner_ = std::make_unique<ManeuverPlanner>(nh_, thread_pool_.get());
    has_maneuver_planner_ = true;
  }
//  auto init_trajectory_point = Planner::GetStitchingTrajectory();
  planning_msgs::TrajectoryPoint init_trajectory_point;
  auto maneuver_status = maneuver_planner_->Process(init_trajectory_point);
  if (maneuver_status != ManeuverStatus::kSuccess) {
    ROS_FATAL("ManeuverPlanner failed, [maneuver_status: %u]", maneuver_status);
  } else {
    ROS_DEBUG("ManeuverPlanner success, [maneuver_status: %u]", maneuver_status);
  }
  auto optimal_trajectory = maneuver_planner_->optimal_trajectory();
  optimal_trajectory.header.stamp = current_time_stamp;
  history_trajectory_ = optimal_trajectory;
  trajectory_publisher_.publish(optimal_trajectory);
  auto valid_trajectories = maneuver_planner_->valid_trajectories();
  if (!valid_trajectories.empty()) {
    this->VisualizeValidTrajectories(valid_trajectories);
  }
}

void Planner::InitPublisher() {
  this->trajectory_publisher_ = nh_.advertise<planning_msgs::Trajectory>(
      topic::kPublishedTrajectoryName, 10);
  this->visualized_trajectory_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      topic::kVisualizedTrajectoryName, 10);
  this->visualized_valid_trajectories_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      topic::kVisualizedValidTrajectoriesName, 10);
}

void Planner::InitSubscriber() {

  this->ego_vehicle_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
      topic::kEgoVehicleStatusName, 10,
      [this](const carla_msgs::CarlaEgoVehicleStatus &ego_vehicle_status) {
        this->ego_vehicle_status_ = ego_vehicle_status;
      });

  this->traffic_lights_subscriber_ = nh_.subscribe<carla_msgs::CarlaTrafficLightStatusList>(
      topic::kTrafficLigthsName, 10,
      [this](const carla_msgs::CarlaTrafficLightStatusList &traffic_light_status_list) {
        this->traffic_light_status_list_ = traffic_light_status_list;
        ROS_INFO("the traffic light status list size: %zu",
                 traffic_light_status_list_.traffic_lights.size());
      });
  this->traffic_lights_info_subscriber_ = nh_.subscribe<carla_msgs::CarlaTrafficLightInfoList>(
      topic::kTrafficLightsInfoName,
      10,
      [this](const carla_msgs::CarlaTrafficLightInfoList &traffic_lights_info_list) {
        traffic_lights_info_list_ = decltype(traffic_lights_info_list_)();
        for (const auto &traffic_light_info : traffic_lights_info_list.traffic_lights) {
          traffic_lights_info_list_.emplace(traffic_light_info.id, traffic_light_info);
        }
      });

  this->ego_vehicle_info_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleInfo>(
      topic::kEgoVehicleInfoName, 10,
      [this](const carla_msgs::CarlaEgoVehicleInfo &ego_vehicle_info) {
        ego_vehicle_info_ = ego_vehicle_info;
        this->ego_vehicle_id_ = ego_vehicle_info_.id;
        ROS_INFO("the ego_vehicle_id_: %i", ego_vehicle_id_);
      });

  this->objects_subscriber_ = nh_.subscribe<derived_object_msgs::ObjectArray>(
      topic::kObjectsName, 10,
      [this](const derived_object_msgs::ObjectArray &object_array) {
        this->objects_map_.clear();
        for (const auto &object : object_array.objects) {
          objects_map_.emplace(object.id, object);
        }
        if (ego_vehicle_id_ != -1) {
          ego_object_ = objects_map_[ego_vehicle_id_];
        }
        ROS_INFO("the objects map_ size is: %lu", objects_map_.size());
      });

  this->ego_vehicle_odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(
      topic::kEgoVehicleOdometryName, 10,
      [this](const nav_msgs::Odometry &odometry) {
        this->ego_odometry_ = odometry;
      });

  this->init_pose_subscriber_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      topic::kInitialPoseName, 10,
      [this](const geometry_msgs::PoseWithCovarianceStamped &init_pose) {
        this->init_pose_ = init_pose;
        PlanningContext::Instance().UpdateGlobalInitPose(init_pose_);
        ROS_INFO("the init_pose_: x: %lf, y: %lf",
                 init_pose_.pose.pose.position.x, init_pose_.pose.pose.position.y);
      });

  this->goal_pose_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      topic::kGoalPoseName, 10,
      [this](const geometry_msgs::PoseStamped &goal_pose) {
        this->goal_pose_ = goal_pose;
        PlanningContext::Instance().UpdateGlobalGoalPose(goal_pose_);
        ROS_INFO("the goal_pose_ : x: %lf, y: %lf",
                 goal_pose_.pose.position.x, goal_pose_.pose.position.y);
      });
  ROS_DEBUG("Planner::InitSubscriber finished");
}

void Planner::InitServiceClient() {
  this->get_waypoint_client_ = nh_.serviceClient<carla_waypoint_types::GetWaypoint>(
      service::kGetEgoWaypontServiceName);
  this->get_actor_waypoint_client_ = nh_.serviceClient<carla_waypoint_types::GetActorWaypoint>(
      service::kGetActorWaypointServiceName);
  ROS_DEBUG("Planner::InitServiceClient finished");
}

bool Planner::UpdateObstacleStatus() {
  /// update the obstacle at each run
  int failed_count = 0;
  std::list<std::shared_ptr<Obstacle>> obstacles;
  for (auto &item : objects_map_) {
    carla_waypoint_types::CarlaWaypoint way_point;
    if (!this->GetWayPoint(item.first, way_point)) {
      failed_count++;
      continue;
    }
    auto obstacle = std::make_shared<Obstacle>(item.second);
    obstacle->set_lane_id(way_point.lane_id);
    obstacle->set_road_id(way_point.road_id);
    obstacle->set_section_id(way_point.section_id);
    obstacles.push_back(obstacle);
  }
  ObstacleFilter::Instance().UpdateObstacles(obstacles);
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
  VehicleState::Instance().set_is_junction(carla_way_point.is_junction);

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
bool Planner::UpdateTrafficLights() {
  auto &instance = TrafficLightList::Instance();

  for (const auto &traffic_light : traffic_light_status_list_.traffic_lights) {
    carla_waypoint_types::CarlaWaypoint waypoint;
    if (!this->GetWayPoint(traffic_light.id, waypoint)) {
      ROS_ERROR("[Planner::UpdateTrafficLights], cannot get the waypoint of traffic light id: %i", traffic_light.id);
      return false;
    }
    carla_msgs::CarlaTrafficLightInfo traffic_light_info = traffic_lights_info_list_[traffic_light.id];
    instance.AddTrafficLight(traffic_light, traffic_light_info, waypoint);
  }
  return true;
}
//planning_msgs::TrajectoryPoint Planner::GetStitchingTrajectory() {
//  planning_msgs::TrajectoryPoint tp;
//  double planning_cycle_time = 1.0 / static_cast<double>(PlanningConfig::Instance().loop_rate());
//  constexpr double kEpsilon_a = 0.4;
//  constexpr double kEpsilon_v = 0.1;
//  const auto &vehicle_state = VehicleState::Instance();
//  if (vehicle_state.linear_vel() < kEpsilon_v && vehicle_state.linear_acc() < kEpsilon_a) {
//    tp.relative_time = planning_cycle_time;
//    tp.path_point.s = 0.0;
//    tp.path_point.x = vehicle_state.pose().position.x;
//    tp.path_point.y = vehicle_state.pose().position.y;
//    tp.path_point.kappa = 0.0;
//    tp.path_point.theta = vehicle_state.theta();
//    tp.path_point.dkappa = 0.0;
//    tp.vel = vehicle_state.linear_vel();
//    tp.acc = vehicle_state.linear_acc();
//    tp.jerk = 0.0;
//    tp.steer_angle = 0.0;
//  } else {
//    tp.relative_time = planning_cycle_time;
//    auto predict_pose = vehicle_state.PredictNextPose(planning_cycle_time);
//    tp.path_point.x = predict_pose.position.x;
//    tp.path_point.y = predict_pose.position.y;
//    tp.path_point.theta = tf::getYaw(predict_pose.orientation);
//    tp.path_point.s = 0.0;
//    tp.path_point.dkappa = 0.0;
//    tp.path_point.kappa = 0.0;
//    tp.vel = std::min(PlanningConfig::Instance().max_lon_velocity(),
//                      vehicle_state.linear_vel() + vehicle_state.linear_acc() * planning_cycle_time);
//    tp.acc = vehicle_state.linear_acc();
//    tp.jerk = 0.0;
//  }
//  return tp;
//}

void Planner::VisualizeValidTrajectories(const std::vector<planning_msgs::Trajectory> &valid_trajectories) const {
  visualization_msgs::MarkerArray valid_trajectories_marker_array;

  for (size_t i = 0; i < valid_trajectories.size(); ++i) {
    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.id = i;
    trajectory_marker.scale.x = 0.5;
    trajectory_marker.color.a = 1.0;
    trajectory_marker.color.r = 1.0;
    trajectory_marker.pose.orientation.w = 1.0;
    trajectory_marker.header.stamp = ros::Time::now();
    trajectory_marker.header.frame_id = "/map";
    trajectory_marker.action = visualization_msgs::Marker::ADD;
    for (const auto &tp : valid_trajectories[i].trajectory_points) {
      geometry_msgs::Point p;
      p.x = tp.path_point.x;
      p.y = tp.path_point.y;
      p.z = 0.1;
      trajectory_marker.points.push_back(p);
    }
    valid_trajectories_marker_array.markers.push_back(trajectory_marker);
  }
  visualized_valid_trajectories_publisher_.publish(valid_trajectories_marker_array);
}

void Planner::VisualizeOptimalTrajectory(const planning_msgs::Trajectory &optimal_trajectory) const {
  visualization_msgs::Marker optimal_trajectory_marker;
  optimal_trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
  optimal_trajectory_marker.header.stamp = ros::Time::now();
  optimal_trajectory_marker.header.frame_id = "/map";
  optimal_trajectory_marker.type = visualization_msgs::Marker::ADD;
  optimal_trajectory_marker.color.a = 1.0;
  optimal_trajectory_marker.color.g = 1.0;
  optimal_trajectory_marker.scale.x = PlanningConfig::Instance().vehicle_params().width;
  optimal_trajectory_marker.pose.orientation.w = 1.0;
  optimal_trajectory_marker.id = 1;
  for (const auto &tp : optimal_trajectory.trajectory_points) {
    geometry_msgs::Point p;
    p.x = tp.path_point.x;
    p.y = tp.path_point.y;
    p.z = 0.1;
    optimal_trajectory_marker.points.push_back(p);
  }
  visualized_trajectory_publisher_.publish(optimal_trajectory_marker);
}
std::vector<planning_msgs::TrajectoryPoint> Planner::GetStitchingTrajectory(const ros::Time &current_time_stamp,
                                                                            double planning_cycle_time,
                                                                            size_t preserve_points_num) {

}

}