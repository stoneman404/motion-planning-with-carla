#include <gtest/gtest.h>
#include <collision_checker/collision_checker.hpp>
#include <obstacle_manager/st_graph.hpp>
#include <tf/transform_datatypes.h>
#include <math/coordinate_transformer.hpp>

using namespace planning;

namespace {
double start_s{2.0};
double end_s{100.0};
double t_start{0.0};
double t_end{8.0};
std::array<double, 3> init_d{1, 0, 0};
double lookahead_time{8.0};
double delta_t{0.1};
vehicle_state::VehicleParams vehicle_params{};
}

TEST(STGraphTEST, st_graph_build) {
  vehicle_params.length = 4.80882;
  vehicle_params.width = 2.16951;
  vehicle_params.front_axle_to_center_length = 1.61906;
  vehicle_params.back_axle_to_center_length = 1.38721;

  size_t number_of_waypoints = 100;
  double heading = 0.0;
  double ds = 2.0;
  std::vector<planning_msgs::WayPoint> way_points;
  way_points.reserve(number_of_waypoints);
  planning_msgs::WayPoint way_point;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  for (size_t i = 0; i < number_of_waypoints; ++i) {
    way_point.s = s;
    way_point.pose.position.x = x;
    way_point.pose.position.y = y;
    way_point.pose.position.z = 0.0;
    way_point.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
    way_point.lane_width = 3.5;
    way_point.lane_id = 1;
    way_point.section_id = 1;
    way_point.road_id = 1;
    way_point.id = i;
    way_point.has_left_lane = false;
    way_point.has_right_lane = false;
    way_point.has_value = true;
    way_point.is_junction = false;
    way_points.push_back(way_point);
    s += ds;
    x += ds * std::cos(heading);
    y += ds * std::sin(heading);
  }
  auto smoothed_ref_line = ReferenceLine(way_points);
  derived_object_msgs::Object object;
  object.object_classified = derived_object_msgs::Object::OBJECT_DETECTED;
  object.classification = derived_object_msgs::Object::CLASSIFICATION_CAR;
  object.shape.type = shape_msgs::SolidPrimitive::BOX;
  object.shape.dimensions.resize(3);
  object.shape.dimensions[0] = 1.0;
  object.shape.dimensions[1] = 3.0;
  object.shape.dimensions[2] = 1.5;
  object.pose.position.x = 10.0;
  object.pose.position.y = 1.0;
  object.pose.position.z = 2.0;
  object.id = 1;
  object.twist.linear.x = 1.0;
  object.twist.linear.y = object.twist.linear.z = 0.0;
  object.twist.angular.x = object.twist.angular.y = object.twist.angular.z = 0.0;
  object.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI_4);
  object.accel.linear.x = object.accel.linear.y = object.accel.linear.z = object.accel.angular.x =
  object.accel.angular.y = object.accel.angular.z = 0.0;
  auto obstacle = std::make_shared<planning::Obstacle>(object);
  obstacle->PredictTrajectory(lookahead_time, delta_t);

  std::vector<std::shared_ptr<planning::Obstacle>> obstacles{obstacle};

  auto st_graph = std::make_shared<planning::STGraph>(
      obstacles, smoothed_ref_line, start_s, end_s,
      t_start, t_end, init_d, 8.0, 0.1);

  std::unordered_map<int, std::shared_ptr<planning::Obstacle>> obstacle_map;
  obstacle_map.emplace(obstacle->Id(), obstacle);
  auto collision_checker = std::make_shared<planning::CollisionChecker>(
      obstacle_map, smoothed_ref_line, st_graph, start_s,
      init_d[0], 1.5, 0.3,
      lookahead_time, 0.1, vehicle_params, nullptr);
  auto path_intervals = st_graph->GetPathBlockingIntervals(t_start, t_end, delta_t);
  int i = 0;
  for (const auto &path_interval : path_intervals) {
    for (const auto& interval : path_interval) {
      std::cout <<  i << "th " <<"start_s: " << interval.first << ", end_s: " << interval.second << std::endl;
    }
    ++i;
  }
  auto st_surrounding_points = st_graph->GetObstacleSurroundingPoints(obstacle->Id(), -1e-2, 0.1);
  for (const auto& st_point : st_surrounding_points) {
    std::cout << "st_point: s: " << st_point.s() << ", t: " << st_point.t() << std::endl;
  }
  s = start_s;
  planning_msgs::Trajectory ego_trajectory;
  double t = 0;
  while (t <= 8.0) {
    auto ref_point = smoothed_ref_line.GetReferencePoint(s);
    auto xy = common::CoordinateTransformer::CalcCatesianPoint(ref_point.theta(), ref_point.x(), ref_point.y(), 0.0);
    planning_msgs::TrajectoryPoint tp;
    tp.path_point.x = xy.x();
    tp.path_point.y = xy.y();
    tp.path_point.theta = ref_point.theta();
    tp.vel = 10.0;
    tp.acc = 0.0;
    tp.jerk = 0.0;
    tp.relative_time = t;
    ego_trajectory.trajectory_points.push_back(tp);
    s += tp.vel * delta_t;
    t += delta_t;
  }
  EXPECT_TRUE( collision_checker->IsCollision(ego_trajectory));
  for (const auto& tp: ego_trajectory.trajectory_points) {
    std::cout << "ego_trajectory: t: " << tp.relative_time <<  " x: " << tp.path_point.x << ", y: " << tp.path_point.y << std::endl;
  }

}