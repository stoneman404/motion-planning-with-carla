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
    auto begin = ros::Time::now();
    ros::spinOnce();
    this->RunOnce();
    auto end = ros::Time::now();
    ROS_INFO("[MotionPlanner::Launch], the RunOnce Elapsed Time: %lf s", (end - begin).toSec());
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
  auto stitching_trajectory =
      this->GetStitchingTrajectory(current_time_stamp,
                                   1.0 / static_cast<double>(PlanningConfig::Instance().loop_rate()),
                                   PlanningConfig::Instance().preserve_history_trajectory_point_num());
  auto init_trajectory_point = stitching_trajectory.back();
  PlanningConfig::Instance().set_vehicle_params(vehicle_state_->vehicle_params());
  std::vector<PlanningTarget> planning_targets;
  planning_msgs::Trajectory optimal_trajectory;

  if (!this->GetPlanningTargetFromBehaviour(behaviour_, planning_targets)) {
    ROS_FATAL("[MotionPlanner::RunOnce], Failed to Get the PlanningTargetFromBehaviours");
    GenerateEmergencyStopTrajectory(init_trajectory_point, optimal_trajectory);
    has_history_trajectory_ = false;
    optimal_trajectory.header.stamp = current_time_stamp;
    optimal_trajectory.status = planning_msgs::Trajectory::EMERGENCYSTOP;
    trajectory_publisher_.publish(optimal_trajectory);
    return;
  }

  if (!trajectory_planner_->Process(init_trajectory_point, planning_targets, optimal_trajectory, nullptr)) {
    GenerateEmergencyStopTrajectory(init_trajectory_point, optimal_trajectory);
    has_history_trajectory_ = false;
    optimal_trajectory.header.stamp = current_time_stamp;
    optimal_trajectory.status = planning_msgs::Trajectory::EMERGENCYSTOP;
    trajectory_publisher_.publish(optimal_trajectory);
    return;
  }

  // because the first point of original optimal_trajectory is init_trajectory_point: the back of stitching_trajectory;
  optimal_trajectory.trajectory_points.insert(optimal_trajectory.trajectory_points.begin(),
                                              stitching_trajectory.begin(),
                                              stitching_trajectory.end() - 1);
  if (optimal_trajectory.trajectory_points.empty()) {
    optimal_trajectory.status = planning_msgs::Trajectory::EMPTY;
  }else{
    optimal_trajectory.status = planning_msgs::Trajectory::NORMAL;
  }
  optimal_trajectory.header.stamp = current_time_stamp;
  history_trajectory_ = optimal_trajectory;
  has_history_trajectory_ = true;
  trajectory_publisher_.publish(optimal_trajectory);
  // for visualization
  VisualizeOptimalTrajectory(optimal_trajectory);
  std::vector<std::shared_ptr<ReferenceLine>> ref_lines;
  for (const auto &target : planning_targets) {
    ref_lines.emplace_back(target.ref_lane);
  }
  VisualizeTrafficLightBox();
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

void MotionPlanner::VisualizeTrafficLightBox() {
  visualization_msgs::MarkerArray traffic_light_boxes_markers;

  for (const auto &traffic_light : traffic_lights_info_list_) {
    if (traffic_light_status_list_.find(traffic_light.first) == traffic_light_status_list_.end()){
      continue;
    }
    if (traffic_light_status_list_[traffic_light.first].state == carla_msgs::CarlaTrafficLightStatus::GREEN){
      continue;
    }
    visualization_msgs::Marker traffic_light_marker;
    traffic_light_marker.type = visualization_msgs::Marker::CUBE;
    traffic_light_marker.header.stamp = ros::Time::now();
    traffic_light_marker.action = visualization_msgs::Marker::ADD;
    traffic_light_marker.header.frame_id = "map";
    traffic_light_marker.color.a = 1.0;
    traffic_light_marker.color.r = 0.8;
    traffic_light_marker.color.g = 0.2;
    traffic_light_marker.color.b = 1.0;
    traffic_light_marker.scale = traffic_light.second.trigger_volume.size;
    traffic_light_marker.pose.position.x = traffic_light.second.trigger_volume.center.x;
    traffic_light_marker.pose.position.y = traffic_light.second.trigger_volume.center.y;
    traffic_light_marker.pose.position.z = traffic_light.second.trigger_volume.center.z;
    traffic_light_marker.pose.orientation = traffic_light.second.transform.orientation;
    traffic_light_marker.id = traffic_light.first;
    traffic_light_boxes_markers.markers.push_back(traffic_light_marker);
  }
  visualized_traffic_light_box_publisher_.publish(traffic_light_boxes_markers);
}

void MotionPlanner::VisualizeReferenceLine(std::vector<std::shared_ptr<ReferenceLine>> &ref_lines) {
  visualization_msgs::MarkerArray marker_array;
  int i = 0;
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
  planning_targets.resize(behaviour.forward_behaviours.size());
  std::vector<std::future<bool>> futures;
  const double distance_weight = PlanningConfig::Instance().reference_smoother_distance_weight();
  const double deviation_weight = PlanningConfig::Instance().reference_smoother_deviation_weight();
  const double heading_weight = PlanningConfig::Instance().reference_smoother_heading_weight();
  const double max_curvature = PlanningConfig::Instance().reference_smoother_max_curvature();

//  for (size_t i = 0; i < behaviour.forward_behaviours.size(); ++i) {
//    const auto lambda =
//        [&i, &planning_targets, &behaviour, &distance_weight, &deviation_weight, &heading_weight, &max_curvature]() -> bool {
//          planning_targets[i].ref_lane = std::make_shared<ReferenceLine>(behaviour.reference_lane[i].way_points);
//          if (!planning_targets[i].ref_lane->Smooth(deviation_weight, heading_weight, distance_weight, max_curvature)) {
//            ROS_FATAL(
//                "[MotionPlanner::GetPlanningTargetFromBehaviour], failed to smooth reference line for lateral_behaviour: %u",
//                behaviour.forward_behaviours[i].behaviour);
//            return false;
//          }
//
//          if (behaviour.forward_behaviours[i].behaviour == behaviour.lat_behaviour.behaviour) {
//            planning_targets[i].is_best_behaviour = true;
//          }
//          planning_targets[i].lateral_behaviour = behaviour.forward_behaviours[i];
//          planning_targets[i].behaviour_trajectory = behaviour.forward_trajectories[i];
//          return true;
//        };
//    futures.push_back(thread_pool_->PushTask(lambda));
//  }
//
//  for (auto &task : futures) {
//    if (!task.get()) {
//      return false;
//    }
//  }

  for (size_t i = 0; i < behaviour.forward_behaviours.size(); ++i) {
    planning_targets[i].ref_lane = std::make_shared<ReferenceLine>(behaviour.reference_lane[i].way_points);
    auto begin = ros::Time::now();
    if (!planning_targets[i].ref_lane->Smooth(deviation_weight, heading_weight, distance_weight, max_curvature)) {
      ROS_FATAL("[MotionPlanner::GetPlanningTargetFromBehaviour], failed to smooth reference line for lateral_behaviour: %u",
          behaviour.forward_behaviours[i].behaviour);
      return false;
    }
    auto end = ros::Time::now();
    ROS_WARN("[GetPlanningTargetFromBehaviour], THE SMOOTH REFERENCELINE ESLAPESD TIME IS : %f s", (end - begin).toSec());

    if (behaviour.forward_behaviours[i].behaviour == behaviour.lat_behaviour.behaviour) {
      planning_targets[i].is_best_behaviour = true;
    }
    planning_targets[i].lateral_behaviour = behaviour.forward_behaviours[i];
    planning_targets[i].behaviour_trajectory = behaviour.forward_trajectories[i];

    MotionPlanner::GetLocalGoal(planning_targets[i]);
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
                                                    planning_msgs::Trajectory &emergency_stop_trajectory) {
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

bool MotionPlanner::GetLocalGoal(PlanningTarget &planning_target) {
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
    ROS_FATAL("[MotionPlanner::GetLocalGoal]: failed to get local goal because get matched point failed");
    return false;
  }
  if (matched_s > ref_lane->Length() - 2.0) {
    planning_target.has_stop_point = true;
    planning_target.stop_s = ref_lane->Length() - PlanningConfig::Instance().lon_safety_buffer();
  } else {
    planning_target.has_stop_point = false;
    planning_target.stop_s = std::numeric_limits<double>::max();
  }

  double v_x = goal_trajectory_point.vel * std::cos(goal_trajectory_point.path_point.theta);
  double v_y = goal_trajectory_point.vel * std::sin(goal_trajectory_point.path_point.theta);
  double ref_v = std::cos(ref_point.theta()) * v_x + std::sin(ref_point.theta()) * v_y;
  double max_ref_v = std::sqrt(PlanningConfig::Instance().max_lat_acc() / (std::fabs(ref_point.kappa()) + 1e-5));
  planning_target.desired_vel = std::min(max_ref_v, ref_v);
//  auto ref_lane = planning_target.ref_lane;
//  double cur_x = planning_target.behaviour_trajectory.trajectory_points.front().path_point.x;
//  double cur_y = planning_target.behaviour_trajectory.trajectory_points.front().path_point.y;
//  ReferencePoint matched_ref_point;
//  double matched_s;
//  if (!ref_lane->GetMatchedPoint(cur_x, cur_y, &matched_ref_point, &matched_s)){
//    return false;
//  }
//  double max_ref_v = std::sqrt(PlanningConfig::Instance().max_lat_acc() / (matched_ref_point.kappa() + 1e-5));
//  if (matched_s + PlanningConfig::Instance().max_lookahead_distance() < ref_lane->Length()){
//    planning_target.has_stop_point = true;
//    planning_target.stop_s = ref_lane->Length();
//  }else{
//    planning_target.has_stop_point = false;
//    planning_target.stop_s = std::numeric_limits<double>::max();
//  }

//  planning_target.desired_vel = std::min(max_ref_v, 10.333);
//  ROS_INFO("[PlanningTarget], desired vel is %lf", ref_v);
//  planning_target.desired_vel = ref_v;
  return true;

}

std::vector<planning_msgs::TrajectoryPoint> MotionPlanner::GetStitchingTrajectory(const ros::Time &current_time_stamp,
                                                                                  double planning_cycle_time,
                                                                                  size_t preserve_points_num) {
  auto state = vehicle_state_->GetKinoDynamicVehicleState();
  if (!has_history_trajectory_) {
    return MotionPlanner::ComputeReinitStitchingTrajectory(planning_cycle_time, state);
  }
  if (history_trajectory_.trajectory_points.empty()) {
    return MotionPlanner::ComputeReinitStitchingTrajectory(planning_cycle_time, state);
  }
  double relative_time = (current_time_stamp - history_trajectory_.header.stamp).toSec();
  auto time_matched_index = GetTimeMatchIndex(relative_time, 1.0e-5, history_trajectory_.trajectory_points);

  // current time smaller than prev first trajectory point's relative time.
  if (time_matched_index == 0 && relative_time < history_trajectory_.trajectory_points.front().relative_time) {
    return MotionPlanner::ComputeReinitStitchingTrajectory(planning_cycle_time, state);
  }
  // current time exceeds the prev last trajectory point's relative time
  if (time_matched_index >= history_trajectory_.trajectory_points.size() - 1) {
    return MotionPlanner::ComputeReinitStitchingTrajectory(planning_cycle_time, state);
  }
  // time matched trajectory point from history trajectory
  auto time_matched_tp = history_trajectory_.trajectory_points[time_matched_index];
  size_t position_matched_index = GetPositionMatchedIndex({state.x, state.y}, history_trajectory_.trajectory_points);
  // position matched trajectory point from history trajectory
  auto position_matched_tp = history_trajectory_.trajectory_points[position_matched_index];
  auto sd = GetLatAndLonDistFromRefPoint(state.x, state.y, position_matched_tp.path_point);
  double lon_diff = time_matched_tp.path_point.s - sd.first;
  double lat_diff = sd.second;
  if (std::abs(lat_diff) > PlanningConfig::Instance().max_replan_lat_distance_threshold()) {
    return MotionPlanner::ComputeReinitStitchingTrajectory(planning_cycle_time, state);
  }
  if (std::abs(lon_diff) > PlanningConfig::Instance().max_replan_lon_distance_threshold()) {
    return MotionPlanner::ComputeReinitStitchingTrajectory(planning_cycle_time, state);
  }
  double forward_rel_time = relative_time + planning_cycle_time;
  size_t forward_rel_matched_index = GetTimeMatchIndex(forward_rel_time, 1.0e-5, history_trajectory_.trajectory_points);

  auto matched_index = std::min(position_matched_index, time_matched_index);
  std::vector<planning_msgs::TrajectoryPoint> stitching_trajectory;
  stitching_trajectory.assign(history_trajectory_.trajectory_points.begin()
                                  + std::max(0, static_cast<int>(matched_index - preserve_points_num)),
                              history_trajectory_.trajectory_points.begin() + forward_rel_matched_index + 1);
  const double zero_s = stitching_trajectory.back().path_point.s;
  for (auto &tp : stitching_trajectory) {
    tp.relative_time = tp.relative_time + (history_trajectory_.header.stamp - current_time_stamp).toSec();
    tp.path_point.s = tp.path_point.s - zero_s;
  }
  return stitching_trajectory;
}

planning_msgs::TrajectoryPoint MotionPlanner::ComputeTrajectoryPointFromVehicleState(double planning_cycle_time,
                                                                                     const vehicle_state::KinoDynamicState &kinodynamic_state) {
  planning_msgs::TrajectoryPoint point;
  point.relative_time = planning_cycle_time;
  point.path_point.x = kinodynamic_state.x;
  point.path_point.y = kinodynamic_state.y;
  point.path_point.s = 0.0;
  point.path_point.theta = kinodynamic_state.theta;
  point.path_point.kappa = kinodynamic_state.kappa;
  point.vel = kinodynamic_state.v;
  point.acc = kinodynamic_state.a;
  point.jerk = 0.0;
  return point;
}
std::vector<planning_msgs::TrajectoryPoint> MotionPlanner::ComputeReinitStitchingTrajectory(double planning_cycle_time,
                                                                                            const vehicle_state::KinoDynamicState &kino_dynamic_state) {
  constexpr double kEpsilon_v = 0.1;
  constexpr double kEpsilon_a = 0.4;
  planning_msgs::TrajectoryPoint reinit_point;
  if (std::fabs(kino_dynamic_state.a) < kEpsilon_a && std::fabs(kino_dynamic_state.v) < kEpsilon_v) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time, kino_dynamic_state);
  } else {
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          kino_dynamic_state.GetNextStateAfterTime(planning_cycle_time));
  }
  return std::vector<planning_msgs::TrajectoryPoint>(1, reinit_point);
}

size_t MotionPlanner::GetPositionMatchedIndex(const std::pair<double, double> &xy,
                                              const std::vector<planning_msgs::TrajectoryPoint> &trajectory) {
  ROS_ASSERT(!trajectory.empty());
  double min_dist_sqr = std::numeric_limits<double>::max();
  size_t min_index = 0;
  constexpr double kEps = 1.0e-5;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    double dist_sqr = (trajectory[i].path_point.x - xy.first) * (trajectory[i].path_point.x - xy.first) +
        (trajectory[i].path_point.y - xy.second) * (trajectory[i].path_point.y - xy.second);
    if (dist_sqr < min_dist_sqr + kEps) {
      min_dist_sqr = dist_sqr;
      min_index = i;
    }
  }
  return min_index;
}

size_t MotionPlanner::GetTimeMatchIndex(double relative,
                                        double eps,
                                        const std::vector<planning_msgs::TrajectoryPoint> &trajectory) {
  ROS_ASSERT(!trajectory.empty());
  if (relative > trajectory.back().relative_time) {
    return trajectory.size() - 1;
  }
  auto cmp = [&eps](const planning_msgs::TrajectoryPoint &tp, double relative_time) -> bool {
    return tp.relative_time + eps < relative_time;
  };
  auto iter = std::lower_bound(trajectory.begin(), trajectory.end(), relative, cmp);
  return std::distance(trajectory.begin(), iter);

}
std::pair<double, double> MotionPlanner::GetLatAndLonDistFromRefPoint(double x,
                                                                      double y,
                                                                      const planning_msgs::PathPoint &point) {
  std::pair<double, double> sd;
  Eigen::Vector2d v{x - point.x, y - point.y};
  Eigen::Vector2d n{std::cos(point.theta), std::sin(point.theta)};
  sd.first = v.x() * n.x() + n.y() + v.y() + point.s;
  sd.second = v.x() * n.y() - v.y() * n.x();
  return sd;
}

}