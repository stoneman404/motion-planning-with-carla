
#include <planning_srvs/Route.h>
#include <memory>
#include <reference_line/reference_line.hpp>
#include <tf/transform_datatypes.h>
#include "maneuver_planner/maneuver_planner.hpp"
#include "maneuver_planner/state.hpp"
#include "maneuver_planner/follow_lane.hpp"
#include "maneuver_planner/change_right_lane.hpp"
#include "maneuver_planner/change_left_lane.hpp"
#include "maneuver_planner/stop.hpp"
#include "maneuver_planner/emergency_stop.hpp"
#include "string_name.hpp"
#include "planning_context.hpp"
#include "planning_config.hpp"
#include "motion_planner/frenet_lattice_planner/frenet_lattice_planner.hpp"

namespace planning {

ManeuverPlanner::ManeuverPlanner(const ros::NodeHandle &nh, ThreadPool *thread_pool)
    : nh_(nh), thread_pool_(thread_pool) {
  this->InitPlanner();
}

void ManeuverPlanner::InitPlanner() {
  trajectory_planner_ = std::make_unique<FrenetLatticePlanner>(thread_pool_);
  valid_trajectories_.clear();
  this->route_service_client_ = nh_.serviceClient<planning_srvs::Route>(
      service::kRouteServiceName);
  auto ego_pose = VehicleState::Instance().pose();
  auto destination = PlanningContext::Instance().global_goal_pose().pose;
  routes_.clear();
  ref_lines_.clear();
  planning_srvs::RouteResponse route_response;
  ROS_ASSERT(this->ReRoute(ego_pose, destination, route_response));
  routes_.push_back(route_response);
  auto ptr_ref_line = std::make_shared<ReferenceLine>();
  ROS_ASSERT(this->GenerateReferenceLine(route_response, ptr_ref_line));
  ref_lines_.push_back(ptr_ref_line);
  current_lane_id_ = VehicleState::Instance().lane_id();
  current_state_.reset(&FollowLane::Instance());
  maneuver_goal_.decision_type = DecisionType::kStopAtDestination;
  maneuver_goal_.maneuver_infos.resize(1);
  maneuver_goal_.maneuver_infos.front().maneuver_target.target_speed = 0.0;
  maneuver_goal_.maneuver_infos.front().ptr_ref_line = ref_lines_.front();
  maneuver_goal_.maneuver_infos.front().has_stop_point = false;
  maneuver_goal_.maneuver_infos.front().lane_id = VehicleState::Instance().lane_id();
  prev_status_ = ManeuverStatus::kUnknown;
  ROS_ASSERT(current_state_->Enter(this));
}

ManeuverStatus ManeuverPlanner::Process(const planning_msgs::TrajectoryPoint &init_trajectory_point) {
  if (current_state_ == nullptr) {
    GenerateEmergencyStopTrajectory(init_trajectory_point, optimal_trajectory_);
    ROS_FATAL("[ManeuverPlanner::Process], the current state is nullptr");
    return ManeuverStatus::kError;
  }
  ROS_DEBUG("[ManeuverPlanner::Process], current state is [%s]", current_state_->Name().c_str());
  ref_lines_.clear();
  if (thread_pool_ != nullptr) {
    std::vector<std::future<std::pair<bool, std::shared_ptr<ReferenceLine>>>> futures;
    for (const auto &route_response : routes_) {
      auto ptr_ref_line = std::make_shared<ReferenceLine>();
      auto task = [this, &route_response, &ptr_ref_line]() -> std::pair<bool, std::shared_ptr<ReferenceLine>> {
        auto result = ManeuverPlanner::GenerateReferenceLine(route_response, ptr_ref_line);
        return std::make_pair(result, ptr_ref_line);
      };
      futures.emplace_back(thread_pool_->Enqueue(task));
    }
    for (auto &future : futures) {
      if (!future.get().first) {
        GenerateEmergencyStopTrajectory(init_trajectory_point, optimal_trajectory_);
        return ManeuverStatus::kError;
      } else {
        ref_lines_.push_back(future.get().second);
      }
    }
  } else {
    for (const auto &route_response : routes_) {
      auto ptr_ref_line = std::make_shared<ReferenceLine>();
      if (!ManeuverPlanner::GenerateReferenceLine(route_response, ptr_ref_line)) {
        GenerateEmergencyStopTrajectory(init_trajectory_point, optimal_trajectory_);
        return ManeuverStatus::kError;
      } else {
        ref_lines_.push_back(ptr_ref_line);
      }
    }
  }

  valid_trajectories_.clear();
  if (maneuver_goal_.decision_type == DecisionType::kEmergencyStop) {
    GenerateEmergencyStopTrajectory(init_trajectory_point, optimal_trajectory_);
    return ManeuverStatus::kSuccess;
  }

  auto trajectory_plan_result = trajectory_planner_->Process(init_trajectory_point,
                                                             maneuver_goal_,
                                                             optimal_trajectory_,
                                                             &valid_trajectories_);
  if (!trajectory_plan_result) {
    GenerateEmergencyStopTrajectory(init_trajectory_point, optimal_trajectory_);
    ROS_FATAL("ManeuverPlanner::Process Failed, [State:%s, init_trajectory_point: {x: %f, y: %f}]",
              current_state_->Name().c_str(), init_trajectory_point.path_point.x,
              init_trajectory_point.path_point.y);
    return ManeuverStatus::kError;
  }
  ROS_DEBUG("[ManeuverPlanner::Process], the size of [valid_trajectories_] is %zu", valid_trajectories_.size());
  std::unique_ptr<State> state(current_state_->Transition(this));
  if (state != nullptr && state->Name() != current_state_->Name()) {
    current_state_->Exit(this);
    current_state_ = std::move(state);
    current_state_->Enter(this);
  }
  return ManeuverStatus::kSuccess;
}

bool ManeuverPlanner::ReRoute(const geometry_msgs::Pose &start,
                              const geometry_msgs::Pose &destination,
                              planning_srvs::RouteResponse &response) {
  planning_srvs::Route srv;
  srv.request.start_pose = start;
  srv.request.end_pose = destination;
  if (!route_service_client_.call(srv)) {
    ROS_FATAL("[Planner::Reroute], Failed to ReRoute!");
    return false;
  } else {
    response = srv.response;
    ROS_DEBUG("[Planner::Reroute], Reroute SUCCESSFUL, the route size is %zu", response.route.size());
    return true;
  }
}

bool ManeuverPlanner::GenerateReferenceLine(const planning_srvs::RouteResponse &route,
                                            std::shared_ptr<ReferenceLine> reference_line) {
  if (reference_line == nullptr) {
    return false;
  }
  const auto vehicle_pose = VehicleState::Instance().pose();
  const double max_forward_distance = PlanningConfig::Instance().reference_max_forward_distance();
  const double max_backward_distance = PlanningConfig::Instance().reference_max_backward_distance();
  const int matched_index = GetNearestIndex(vehicle_pose, route.route);
  const int start_index = GetStartIndex(matched_index, max_backward_distance, route.route);
  const int end_index = GetEndIndex(matched_index, max_forward_distance, route.route);
  if (end_index - start_index < 1) {
    return false;
  }
  const auto sample_way_points = GetWayPointsFromStartToEndIndex(start_index, end_index, route.route);
  if (sample_way_points.empty()) {
    ROS_FATAL("[GenerateReferenceLine], failed to get sampled way points");
    return false;
  }
  reference_line.reset(new ReferenceLine(route.route));
  if (!reference_line->Smooth()) {
    ROS_DEBUG("[ManeuverPlanner::GenerateReferenceLine], failed to smooth reference line.");
  }
  return true;
}

int ManeuverPlanner::GetNearestIndex(const geometry_msgs::Pose &ego_pose,
                                     const std::vector<planning_msgs::WayPoint> &way_points) {

  double min_distance = std::numeric_limits<double>::max();
  const double ego_x = ego_pose.position.x;
  const double ego_y = ego_pose.position.y;
  int min_index = 0;
  for (size_t i = 0; i < way_points.size(); ++i) {
    double x = way_points[i].pose.position.x;
    double y = way_points[i].pose.position.y;
    double dist = std::hypot(x - ego_x, y - ego_y);
    if (dist < min_distance) {
      min_distance = dist;
      min_index = i;
    }
  }
  return min_index;
}

int ManeuverPlanner::GetStartIndex(const int matched_index, double backward_distance,
                                   const std::vector<planning_msgs::WayPoint> &way_points) {
  int current_index = matched_index;
  const auto matched_way_point = way_points[current_index];
  double s = 0;
  int last_index = current_index;

  while (current_index > 0 && s < backward_distance) {
    last_index = current_index - 1;
    const auto last_way_point = way_points[last_index];
    double ds = std::hypot(last_way_point.pose.position.x - matched_way_point.pose.position.x,
                           last_way_point.pose.position.y - matched_way_point.pose.position.y);
    s += ds;
    current_index--;
  }
  return last_index;
}

int ManeuverPlanner::GetEndIndex(const int matched_index,
                                 double forward_distance,
                                 const std::vector<planning_msgs::WayPoint> &way_points) {
  int current_index = matched_index;
  const auto matched_way_point = way_points[current_index];
  double s = 0;
  int next_index = current_index;
  while (current_index < way_points.size() - 2 && s < forward_distance) {
    next_index = current_index + 1;
    const auto last_way_point = way_points[next_index];
    double ds = std::hypot(last_way_point.pose.position.x - matched_way_point.pose.position.x,
                           last_way_point.pose.position.y - matched_way_point.pose.position.y);
    s += ds;
    current_index++;
  }

  return next_index;
}

std::vector<planning_msgs::WayPoint> ManeuverPlanner::GetWayPointsFromStartToEndIndex(
    const int start_index,
    const int end_index,
    const std::vector<planning_msgs::WayPoint> &way_points) {
  if (start_index >= end_index) {
    ROS_ERROR("[ManeuverPlanner::GetWayPointsFromStartToEndIndex], "
              "Failed to get waypoints because start_index >= end_index");
    return {};
  }
  std::vector<planning_msgs::WayPoint> sampled_way_points;
  for (size_t i = start_index; i <= end_index; ++i) {
    sampled_way_points.push_back(way_points[i]);
  }
  return sampled_way_points;
}

ManeuverPlanner::~ManeuverPlanner() {
  current_state_->Exit(this);
  current_state_.reset(nullptr);
}

void ManeuverPlanner::SetManeuverGoal(const ManeuverGoal &maneuver_goal) {
  this->maneuver_goal_ = maneuver_goal;
}

const ManeuverGoal &ManeuverPlanner::maneuver_goal() const {
  return maneuver_goal_;
}

ManeuverGoal &ManeuverPlanner::multable_maneuver_goal() { return maneuver_goal_; }

int ManeuverPlanner::GetLaneId() const { return current_lane_id_; }

std::list<planning_srvs::RouteResponse> &ManeuverPlanner::multable_routes() { return routes_; }

std::list<std::shared_ptr<ReferenceLine>> &ManeuverPlanner::multable_ref_line() { return ref_lines_; }

void ManeuverPlanner::GenerateEmergencyStopTrajectory(const planning_msgs::TrajectoryPoint &init_trajectory_point,
                                                      planning_msgs::Trajectory &emergency_trajectory) {
  const double kMaxTrajectoryTime = PlanningConfig::Instance().max_lookahead_time();
  const double kTimeGap = PlanningConfig::Instance().delta_t();
  emergency_trajectory.trajectory_points.clear();
  auto num_traj_point = static_cast<int>(kMaxTrajectoryTime / kTimeGap);
  emergency_trajectory.trajectory_points.resize(num_traj_point);
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
  emergency_trajectory.trajectory_points.push_back(tp);
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
    emergency_trajectory.trajectory_points.push_back(tp);
    last_s = tp.path_point.s;
    last_x = tp.path_point.x;
    last_y = tp.path_point.y;
    last_theta = tp.path_point.theta;
    last_v = tp.vel;
    last_a = tp.acc;
  }

}
const std::vector<planning_msgs::Trajectory> &ManeuverPlanner::valid_trajectories() const {
  return valid_trajectories_;
}

const planning_msgs::Trajectory &ManeuverPlanner::optimal_trajectory() const {
  return optimal_trajectory_;
}

}