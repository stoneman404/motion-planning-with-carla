#include "collision_checker/collision_checker.hpp"
#include <utility>

namespace planning {
using namespace vehicle_state;
using namespace common;
CollisionChecker::CollisionChecker(const std::unordered_map<int, std::shared_ptr<Obstacle>> &obstacles,
                                   std::shared_ptr<ReferenceLine> ptr_ref_line,
                                   std::shared_ptr<STGraph> ptr_st_graph,
                                   double ego_vehicle_s,
                                   double ego_vehicle_d,
                                   double lon_buffer,
                                   double lat_buffer,
                                   double lookahead_time,
                                   double delta_t,
                                   const VehicleParams& vehicle_params,
                                   ThreadPool *thread_pool)
    : ptr_ref_line_(std::move(ptr_ref_line)),
      ptr_st_graph_(std::move(ptr_st_graph)),
      thread_pool_(thread_pool),
      lon_buffer_(lon_buffer),
      lat_buffer_(lat_buffer),
      lookahead_time_(lookahead_time),
      delta_t_(delta_t){
  predicted_obstacle_box_.clear();
  this->Init(obstacles, ego_vehicle_s, ego_vehicle_d, ptr_ref_line_);
}

bool CollisionChecker::IsCollision(const planning_msgs::Trajectory &trajectory) const {
  double ego_width = vehicle_params_.width;
  double ego_length = vehicle_params_.length;
  double shift_distance = vehicle_params_.back_axle_to_center_length;
  if (predicted_obstacle_box_.empty()){
    return true;
  }
  if (this->thread_pool_ == nullptr) {
    for (size_t i = 0; i < trajectory.trajectory_points.size(); ++i) {
      const auto &traj_point = trajectory.trajectory_points.at(i);
      double ego_theta = traj_point.path_point.theta;
      Box2d ego_box = Box2d({traj_point.path_point.x, traj_point.path_point.y}, ego_theta, ego_length, ego_width);
      ego_box.Shift({shift_distance * std::cos(ego_theta), shift_distance * std::sin(ego_theta)});
      for (const auto &obstacle_box : predicted_obstacle_box_[i]) {
        if (ego_box.HasOverlapWithBox2d(obstacle_box)) {
          return true;
        }
      }
    }
    return false;
  } else {
    std::vector<std::future<bool>> futures;
//    double shift_distance = vehicle_params_.back_axle_to_center_length;
    for (size_t i = 0; i < trajectory.trajectory_points.size(); ++i) {
      const auto &traj_point = trajectory.trajectory_points[i];
      double ego_theta = traj_point.path_point.theta;
      Box2d ego_box = Box2d({traj_point.path_point.x, traj_point.path_point.y}, ego_theta, ego_length, ego_width);
      for (const auto &obstacle_box : predicted_obstacle_box_[i]) {
        const auto task = [&ego_box, &obstacle_box]() -> bool {
          if (ego_box.HasOverlapWithBox2d(obstacle_box)) {
            return true;
          }
          return false;
        };
        futures.emplace_back(thread_pool_->PushTask(task));
      }

    }
    for (auto &future : futures) {
      if (future.get()) {
        return true;
      }
    }
    return false;
  }

}
void CollisionChecker::Init(const std::unordered_map<int, std::shared_ptr<Obstacle>> &obstacles,
                            double ego_vehicle_s,
                            double ego_vehicle_d,
                            const std::shared_ptr<ReferenceLine> &reference_line) {

  bool ego_vehicle_in_lane = IsEgoVehicleInLane(ego_vehicle_s, ego_vehicle_d);
  std::vector<std::shared_ptr<Obstacle>> obstacle_considered;
  for (auto &obstacle : obstacles) {
    if (ego_vehicle_in_lane &&
        (IsObstacleBehindEgoVehicle(obstacle.second, ego_vehicle_s, reference_line)
            || !ptr_st_graph_->IsObstacleInGraph(obstacle.first))) {
      continue;
    }
    obstacle_considered.push_back(obstacle.second);
  }

  double relative_time = 0.0;
  while (relative_time < lookahead_time_) {
    std::vector<Box2d> predicted_env;
    for (const auto &obstacle : obstacle_considered) {
      planning_msgs::TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBoxAtPoint(point);
      box.LateralExtend(2.0 * lat_buffer_);
      box.LongitudinalExtend(2.0 * lon_buffer_);
      predicted_env.push_back(box);
    }
    predicted_obstacle_box_.push_back(std::move(predicted_env));
    relative_time +=delta_t_;
  }
}

bool CollisionChecker::IsEgoVehicleInLane(double ego_vehicle_s, double ego_vehicle_d) const {
  double left_width = 0.0;
  double right_width = 0.0;
  ptr_ref_line_->GetLaneWidth(ego_vehicle_s, &left_width, &right_width);
  return ego_vehicle_d < left_width && ego_vehicle_d > -right_width;
}

bool CollisionChecker::IsObstacleBehindEgoVehicle(const std::shared_ptr<Obstacle> &obstacle,
                                                  double ego_s,
                                                  const std::shared_ptr<ReferenceLine> &ref_line) const {
  double default_lane_width = 3.0;
  planning_msgs::TrajectoryPoint point = obstacle->GetPointAtTime(0.0);
  ReferencePoint matched_ref_point;
  double matched_s;
  SLPoint sl_point;
  ptr_ref_line_->XYToSL(point.path_point.x, point.path_point.y, &sl_point);
  if (ego_s > sl_point.s && std::fabs(sl_point.l) < default_lane_width / 2.0) {
    return true;
  }
  return false;
}
}