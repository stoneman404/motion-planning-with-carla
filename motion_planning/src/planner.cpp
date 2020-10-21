#include <geometry_msgs/PoseStamped.h>
#include "planner.hpp"
#include "string_name.hpp"
#include "planning_config.hpp"
#include "vehicle_state/vehicle_state.hpp"
#include "obstacle_filter/obstacle.hpp"
#include "obstacle_filter/obstacle_filter.hpp"
#include "planning_context.hpp"
#include "traffic_lights/traffic_light_list.hpp"

namespace planning {
Planner::Planner(const ros::NodeHandle &nh) : nh_(nh) {
  PlanningConfig::Instance().UpdateParams(nh_);
  this->InitPublisher();
  this->InitSubscriber();
  this->InitServiceClient();
  ROS_INFO("**Planner**");
}

void Planner::MainLoop() {
  ros::Rate loop_rate(PlanningConfig::Instance().loop_rate());
  while (ros::ok()) {
    std::cout << "loop rate: " << PlanningConfig::Instance().loop_rate() << std::endl;
    this->RunOnce();
    ros::spinOnce();
    loop_rate.sleep();
  }
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
    maneuver_planner_ = std::make_unique<ManeuverPlanner>(nh_);
    has_maneuver_planner_ = true;
  }
  auto init_trajectory_point = this->GetInitTrajectoryPoint();
  maneuver_planner_->Process(init_trajectory_point);

}

void Planner::InitPublisher() {
  this->trajectory_publisher_ = nh_.advertise<planning_msgs::Trajectory>(
      topic::kPublishedTrajectoryName, 10);
  this->visualized_trajectory_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      topic::kVisualizedTrajectoryName, 10);
  ROS_DEBUG("Planner::InitPublisher finished");
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
        ROS_INFO("the traffic light status list size: %zu",
                 traffic_light_status_list_.traffic_lights.size());
      });
  this->traffic_lights_info_subscriber_ = nh_.subscribe<carla_msgs::CarlaTrafficLightInfoList>(
      topic::kTrafficLightsInfoName,
      10,
      [this](const carla_msgs::CarlaTrafficLightInfoList::ConstPtr traffic_lights_info_list) {
        traffic_lights_info_list_ = decltype(traffic_lights_info_list_)();
        for (const auto &traffic_light_info : traffic_lights_info_list->traffic_lights) {
          traffic_lights_info_list_.emplace(traffic_light_info.id, traffic_light_info);
        }
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

/**
 *  第二个问题：每个实时规划的初始状态量，比如 s、速度、加速度等是以车体底盘实时反馈为主还是从组合导航获得，
 *  还是说通过一定方式从上帧规划结果获得参考量？交给控制去执行的连续两帧轨迹如何联系起来，才能保证控制模块在连接处速度、加速度、曲率等不发生突变？
 *  A2:这个问题非常好，在今天的分享中没有专门的介绍。我这里简要描述一下，车辆的状态是由上游的定位模块获得的，融合了多种传感器的数据，
 *  包括当前地图坐标系下的坐标，朝向，转向角度，速度，加速度等等。轨迹规划模块以固定的频率进行，我们使用了轨迹拼接的算法（Trajectory Stitching）
 *  保证相邻帧的轨迹在控制器看来是平滑的。假设我们的周期时间是dt秒，如果我们没有上一周期的轨迹，那我们使用运动学模型，
 *  对当前从定位模块获得的车辆状态进行外推，获得dt时间之后的状态作为规划起始点，我们称之为重新规划（Replan）；
 *  如果上一周期的轨迹存在，我们会根据当前系统时间T，在上一周期的轨迹中找到相对应的轨迹点，然后，我们进行一个比较，
 *  比较这个轨迹点与定位模块获得的当前车辆状态的差异，如果这个差异在一定范围内，我们找到T+dt时间的上一周期轨迹点作为规划起始点；
 *  如果这个差异超过设定范围，说明控制器有了较大的误差，我们会做第一种情况的replan。
 *  这种机制保证了在控制误差允许的情况下，做到相邻帧轨迹的平滑拼接。在控制器看起来，规划模块发出的轨迹是一小段一小段dt长度的轨迹光滑拼接起来的。
 *  补充问题：为什么每次规划时，不以车辆当前的状态为规划起始点呢，而是“找到T+dt时间的上一周期轨迹点作为规划起始点”？
 *  因为规划结果真正送到控制器是T+dt时刻
 */
planning_msgs::TrajectoryPoint Planner::GetInitTrajectoryPoint() {
  planning_msgs::TrajectoryPoint init_trajectory_point;
  if (!has_history_trajectory_) {
    double ego_x = VehicleState::Instance().pose().position.x;
    double ego_y = VehicleState::Instance().pose().position.y;
    double ego_theta = VehicleState::Instance().theta();
    double center_to_back_axle = PlanningConfig::Instance().vehicle_params().back_axle_to_center_length;
    init_trajectory_point.path_point.x = ego_x - center_to_back_axle * std::cos(ego_theta);
    init_trajectory_point.path_point.y = ego_y - center_to_back_axle * std::sin(ego_theta);
    init_trajectory_point.path_point.s = 0.0;
    init_trajectory_point.path_point.kappa =
    init_trajectory_point.vel = VehicleState::Instance().linear_vel();
    init_trajectory_point.acc = VehicleState::Instance().linear_acc();
    init_trajectory_point.relative_time = 0.0;
    init_trajectory_point.jerk = 0.0;
  }
}

}