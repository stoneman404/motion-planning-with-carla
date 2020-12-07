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

  if (PlanningConfig::Instance().planner_type() == "frenet_lattice") {
    trajectory_planner_ = std::make_unique<FrenetLatticePlanner>(thread_pool_.get());
  } else {
    ROS_FATAL("MotionPlanner, no such [%s] trajectory planner at now",
              PlanningConfig::Instance().planner_type().c_str());
    ROS_ASSERT(false);
  }

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
  auto init_trajectory_point =
      this->GetStitchingTrajectory(current_time_stamp,
                                   1.0 / PlanningConfig::Instance().loop_rate(),
                                   5).front();
  PlanningConfig::Instance().set_vehicle_params(vehicle_state_->vehicle_params());
  std::vector<PlanningTarget> planning_targets;
  planning_msgs::Trajectory optimal_trajectory;

  if (!this->GetPlanningTargetFromBehaviour(behaviour_, planning_targets)) {
    ROS_FATAL("[MotionPlanner::RunOnce], Failed to Get the PlanningTargetFromBehaviours");
    GenerateEmergencyStopTrajectory(init_trajectory_point, optimal_trajectory);
    trajectory_publisher_.publish(optimal_trajectory);
    return;
  }
  if (!trajectory_planner_->Process(init_trajectory_point, planning_targets, optimal_trajectory, nullptr)) {
    GenerateEmergencyStopTrajectory(init_trajectory_point, optimal_trajectory);
    trajectory_publisher_.publish(optimal_trajectory);
    return;
  }
  VisualizeOptimalTrajectory(optimal_trajectory);
  std::vector<std::shared_ptr<ReferenceLine>> ref_lines;
  for (const auto &target : planning_targets) {
    ref_lines.emplace_back(target.ref_lane);
  }
  VisualizeReferenceLine(ref_lines);
}

void MotionPlanner::InitPublisher() {
  this->trajectory_publisher_ = nh_.advertise<planning_msgs::Trajectory>(
      common::topic::kPublishedTrajectoryName, 1);
  this->visualized_trajectory_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      common::topic::kVisualizedTrajectoryName, 1);
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
      common::topic::kBehaviourName, 5,
      [this](const planning_msgs::Behaviour::ConstPtr &behaviour) {
        this->behaviour_ = *behaviour;
      });
  this->ego_vehicle_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
      common::topic::kEgoVehicleStatusName, 5,
      [this](const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &ego_vehicle_status) {
        this->ego_vehicle_status_ = *ego_vehicle_status;
      });
  this->traffic_lights_subscriber_ = nh_.subscribe<carla_msgs::CarlaTrafficLightStatusList>(
      common::topic::kTrafficLigthsStatusName, 5,
      [this](const carla_msgs::CarlaTrafficLightStatusList::ConstPtr &traffic_light_status_list) {
        traffic_light_status_list_ = decltype(traffic_light_status_list_)();
        for (const auto &traffic_light_status : traffic_light_status_list->traffic_lights) {
          traffic_light_status_list_.emplace(traffic_light_status.id, traffic_light_status);
        }
      });
  this->traffic_lights_info_subscriber_ = nh_.subscribe<carla_msgs::CarlaTrafficLightInfoList>(
      common::topic::kTrafficLightsInfoName, 5,
      [this](const carla_msgs::CarlaTrafficLightInfoList::ConstPtr &traffic_lights_info_list) {
        traffic_lights_info_list_ = decltype(traffic_lights_info_list_)();
        for (const auto &traffic_light_info : traffic_lights_info_list->traffic_lights) {
          traffic_lights_info_list_.emplace(traffic_light_info.id, traffic_light_info);
        }
      });
  this->ego_vehicle_info_subscriber_ = nh_.subscribe<carla_msgs::CarlaEgoVehicleInfo>(
      common::topic::kEgoVehicleInfoName, 5,
      [this](const carla_msgs::CarlaEgoVehicleInfo::ConstPtr &ego_vehicle_info) {
        ego_vehicle_info_ = *ego_vehicle_info;
        this->ego_vehicle_id_ = ego_vehicle_info_.id;
        ROS_INFO("the ego_vehicle_id_: %i", ego_vehicle_id_);
      });
  this->objects_subscriber_ = nh_.subscribe<derived_object_msgs::ObjectArray>(
      common::topic::kObjectsName, 5,
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
  return behaviour_.forward_trajectories.begin()->trajectory_points;

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
bool MotionPlanner::GetPlanningTargetFromBehaviour(const planning_msgs::Behaviour &behaviour,
                                                   std::vector<PlanningTarget> &planning_targets) {
  if (behaviour.lat_behaviour.behaviour == planning_msgs::LateralBehaviour::UNDEFINED) {
    ROS_FATAL("[MotionPlanner::GetPlanningTargetFromBehaviour], the lateral behaviour is UNDEFINED");
    return false;
  }
  planning_targets.clear();
  planning_targets.reserve(behaviour.forward_behaviours.size());
  std::vector<std::future<bool>> futures;
  const double distance_weight = PlanningConfig::Instance().reference_smoother_distance_weight();
  const double deviation_weight = PlanningConfig::Instance().reference_smoother_deviation_weight();
  const double heading_weight = PlanningConfig::Instance().reference_smoother_heading_weight();
  const double max_curvature = PlanningConfig::Instance().reference_smoother_max_curvature();

  for (size_t i = 0; i < behaviour.forward_behaviours.size(); ++i) {
    const auto lambda =
        [&i, &planning_targets, &behaviour, &distance_weight, &deviation_weight, &heading_weight, &max_curvature]() -> bool {
          planning_targets[i].ref_lane = std::make_shared<ReferenceLine>(behaviour.reference_lane[i].way_points);
          if (!planning_targets[i].ref_lane->Smooth(deviation_weight, heading_weight, distance_weight, max_curvature)) {
            ROS_FATAL(
                "[MotionPlanner::GetPlanningTargetFromBehaviour], failed to smooth reference line for lateral_behaviour: %u",
                behaviour.forward_behaviours[i].behaviour);
            return false;
          }

          if (behaviour.forward_behaviours[i].behaviour == behaviour.lat_behaviour.behaviour) {
            planning_targets[i].is_best_behaviour = true;
          }
          planning_targets[i].lateral_behaviour = behaviour.forward_behaviours[i];
          planning_targets[i].behaviour_trajectory = behaviour.forward_trajectories[i];
          return true;
        };
    futures.push_back(thread_pool_->PushTask(lambda));
  }

  for (auto &task : futures) {
    if (!task.get()) {
      return false;
    }
  }

  for (size_t i = 0; i < behaviour.forward_behaviours.size(); ++i) {
    this->GetLocalGoal(planning_targets[i]);
    for (size_t j = 0; j < behaviour.surroud_trajectories[i].trajectories.size(); ++i) {
      int agent_id = behaviour.surroud_trajectories[i].trajectories[j].id;
      if (objects_map_.find(agent_id) != objects_map_.end()) {
        planning_targets[i].obstacles.emplace_back(std::make_shared<Obstacle>(objects_map_[agent_id]));
        planning_targets[i].obstacles.back()->SetTrajectory(behaviour.surroud_trajectories[i].trajectories[j].trajectory);
      } else if (traffic_lights_info_list_.find(agent_id) != traffic_lights_info_list_.end()) {
        planning_targets[i].obstacles.emplace_back(std::make_shared<Obstacle>(traffic_lights_info_list_[i],
                                                                              traffic_light_status_list_[i]));
        planning_targets[i].obstacles.back()->SetTrajectory(behaviour.surroud_trajectories[i].trajectories[j].trajectory);
      } else {
        continue;
      }
    }
  }
  return true;
}
void MotionPlanner::GenerateEmergencyStopTrajectory(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                                    planning_msgs::Trajectory &emergency_stop_trajectory) const {
  const double kMaxTrajectoryTime = PlanningConfig::Instance().max_lookahead_time();
  const double kTimeGap = PlanningConfig::Instance().delta_t();
  emergency_stop_trajectory.trajectory_points.clear();
  auto num_traj_point = static_cast<int>(kMaxTrajectoryTime / kTimeGap);
  emergency_stop_trajectory.trajectory_points.resize(num_traj_point);
  const double max_decel = PlanningConfig::Instance().max_lon_acc();
  double stop_time = init_trajectory_point.vel / max_decel;
  double last_x = init_trajectory_point.path_point.x;
  double last_y = init_trajectory_point.path_point.y;
  double last_v = init_trajectory_point.vel;
  double last_a = max_decel;
  double last_theta = init_trajectory_point.path_point.theta;
  double last_s = init_trajectory_point.path_point.s;
  planning_msgs::TrajectoryPoint tp;
  tp = init_trajectory_point;
  tp.relative_time = 0.0;
  tp.acc = -max_decel;
  emergency_stop_trajectory.trajectory_points.push_back(tp);
  for (int i = 1; i < num_traj_point; ++i) {
    double t = i * kTimeGap + init_trajectory_point.relative_time;
    tp.relative_time = t;
    tp.vel = init_trajectory_point.vel - last_a * kTimeGap;
    tp.acc = t <= stop_time ? -max_decel : 0.0;
    tp.jerk = 0.0;
    double ds = (0.5 * last_a * kTimeGap + last_v) * kTimeGap;
    tp.path_point.x = last_x + std::cos(last_theta) * ds;
    tp.path_point.y = last_y + std::sin(last_theta) * ds;
    tp.path_point.theta = last_theta;
    tp.path_point.s = last_s + ds;
    tp.path_point.dkappa = 0.0;
    tp.path_point.kappa = 0.0;
    tp.steer_angle = init_trajectory_point.steer_angle;
    emergency_stop_trajectory.trajectory_points.push_back(tp);
    last_s = tp.path_point.s;
    last_x = tp.path_point.x;
    last_y = tp.path_point.y;
    last_theta = tp.path_point.theta;
    last_v = tp.vel;
    last_a = tp.acc;
  }
}

bool MotionPlanner::GetLocalGoal(PlanningTarget &planning_target) const {
  auto trajectory = planning_target.behaviour_trajectory;
  double t = PlanningConfig::Instance().max_lookahead_time();
  planning_msgs::TrajectoryPoint goal_trajectory_point;
  auto matched_iterator = std::lower_bound(trajectory.trajectory_points.begin(), trajectory.trajectory_points.end(), t,
                                           [](const planning_msgs::TrajectoryPoint &p0,
                                              const double relative_time) -> bool {
                                             return p0.relative_time < relative_time;
                                           });
  if (matched_iterator == trajectory.trajectory_points.begin()) {
    goal_trajectory_point = trajectory.trajectory_points.front();
  } else if (matched_iterator == trajectory.trajectory_points.end()) {
    goal_trajectory_point = trajectory.trajectory_points.back();
  } else {
    goal_trajectory_point = common::MathUtils::InterpolateTrajectoryPoint(*(matched_iterator - 1),
                                                                          *matched_iterator,
                                                                          PlanningConfig::Instance().max_lookahead_time());
  }
  auto ref_lane = planning_target.ref_lane;
  ReferencePoint ref_point;
  double matched_s;
  if (!ref_lane->GetMatchedPoint(goal_trajectory_point.path_point.x,
                                 goal_trajectory_point.path_point.y,
                                 &ref_point,
                                 &matched_s)) {
    return false;
  }
  if (matched_s > ref_lane->Length() - 2.0) {
    planning_target.has_stop_point = true;
    planning_target.stop_s = std::min(ref_lane->Length(), matched_s);
  } else {
    planning_target.has_stop_point = false;
  }

  double v_x = goal_trajectory_point.vel * std::cos(goal_trajectory_point.path_point.theta);
  double v_y = goal_trajectory_point.vel * std::sin(goal_trajectory_point.path_point.theta);
  double ref_v = std::cos(ref_point.heading()) * v_x + std::sin(ref_point.heading()) * v_y;
  double max_ref_v = std::sqrt(PlanningConfig::Instance().max_lat_acc() * ref_point.kappa());
  planning_target.desired_vel = std::min(ref_v, max_ref_v);
  return true;
}

}