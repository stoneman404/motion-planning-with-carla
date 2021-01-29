#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_REFERENCE_GENERATOR_REFERENCE_GENERATOR_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_REFERENCE_GENERATOR_REFERENCE_GENERATOR_HPP_
#include "reference_line/reference_line.hpp"
#include "vehicle_state/vehicle_state.hpp"
#include <mutex>
#include <boost/circular_buffer.hpp>
#include <future>
#include <tf/transform_datatypes.h>
namespace planning {

struct ReferenceLineConfig {
  ReferenceLineConfig()
      : reference_smooth_max_curvature_(0.0),
        reference_smooth_deviation_weight_(0.0),
        reference_smooth_heading_weight_(0.0),
        reference_smooth_length_weight_(0.0),
        reference_smooth_slack_weight_(0.0) {}
  double reference_smooth_max_curvature_{0.0};
  double reference_smooth_deviation_weight_{0.0};
  double reference_smooth_heading_weight_{0.0};
  double reference_smooth_length_weight_{0.0};
  double reference_smooth_slack_weight_{0.0};

};

struct RouteInfo {
  std::vector<planning_msgs::WayPoint> main_lane;
  std::vector<std::vector<planning_msgs::WayPoint>> left_lanes;
  std::vector<std::vector<planning_msgs::WayPoint>> right_lanes;
  void PrintRouteInfo() {
    std::cout << "=============== main_lane: ============" << std::endl;
    for (const auto &waypoint : main_lane) {
      std::cout << "x: " << waypoint.pose.position.x << ", y: " << waypoint.pose.position.y << ", theta: "
                << tf::getYaw(waypoint.pose.orientation) << std::endl;
    }
    std::cout << "============ left lane: ================" << std::endl;
    int i = 0;
    for (const auto &lane : left_lanes) {
      std::cout << "lane " << i << std::endl;
      for (const auto &waypoint : lane) {
        std::cout << "x: " << waypoint.pose.position.x << ", y: " << waypoint.pose.position.y << ", theta: "
                  << tf::getYaw(waypoint.pose.orientation) << std::endl;
      }
      i++;
    }

    std::cout << "============ right lane: ================" << std::endl;
    int j = 0;
    for (const auto &lane : right_lanes) {
      std::cout << "lane " << j << std::endl;
      for (const auto &waypoint : lane) {
        std::cout << "x: " << waypoint.pose.position.x << ", y: " << waypoint.pose.position.y << ", theta: "
                  << tf::getYaw(waypoint.pose.orientation) << std::endl;
      }
      j++;
    }
  }
};


class ReferenceGenerator {
 public:
  ReferenceGenerator() = default;
  ~ReferenceGenerator() = default;
  ReferenceGenerator(const ReferenceLineConfig &config, double lookahead_distance, double lookback_distace);

  bool Start();
  void Stop();
  bool UpdateRouteResponse(const planning_srvs::RoutePlanServiceResponse &route_response);
  bool UpdateVehicleState(const vehicle_state::KinoDynamicState &vehicle_state);
  bool GetReferenceLines(std::vector<ReferenceLine> *reference_lines);
  bool UpdateReferenceLine(const std::vector<ReferenceLine> &reference_lines);
  void GenerateThread();

  /**
   * @brief:
   * @param[out] ref_lanes, the reference line vector, ref_lanes[0]: the main reference line, if the ref_lanes.size() > 1,
   * then there're changeable lanes, right lane or/and left lane
   * @return: true if this procedure is successful.
   */
  bool CreateReferenceLines(bool smooth, std::vector<ReferenceLine> &ref_lanes);

  /**
   * @param vehicle_state
   * @param lane
   * @param lookahead_distance
   * @param lookback_distance
   * @param smooth
   * @param max_curvature
   * @param deviation_weight
   * @param heading_weight
   * @param length_weight
   * @param ref_lane
   * @return
   */
  static bool RetriveReferenceLine(ReferenceLine &ref_lane,
                                   const vehicle_state::KinoDynamicState &vehicle_state,
                                   const std::vector<planning_msgs::WayPoint> &lane,
                                   double lookahead_distance,
                                   double lookback_distance,
                                   bool smooth = false,
                                   const ReferenceLineConfig &smooth_config = ReferenceLineConfig());

 private:
  /**
   * @brief: has overlap with ref lane along s direction?
   * @param ref_lane
   * @param waypoints
   * @param overlap_start_s
   * @param overlap_end_s
   * @return
   */
  static bool HasOverLapWithRefLane(const ReferenceLine &ref_lane,
                                    std::vector<planning_msgs::WayPoint> &waypoints,
                                    double *overlap_start_s,
                                    double *overlap_end_s);

  /**
   * @brief: split the raw reference line
   * @param raw_lane
   * @return
   */
  static std::vector<std::vector<planning_msgs::WayPoint>> SplitRawLane(const planning_msgs::Lane &raw_lane);

 private:
  bool is_initialized_ = false;
  std::atomic<bool> is_stop_{false};
  ReferenceLineConfig smooth_config_;
  double lookahead_distance_{};
  double lookback_distance_{};
  std::mutex route_mutex_;
  RouteInfo route_info_;
  bool has_route_ = false;
  bool has_vehicle_state_ = false;
  std::mutex vehicle_mutex_;
  vehicle_state::KinoDynamicState vehicle_state_{};
  std::mutex reference_line_mutex_;
  std::vector<ReferenceLine> ref_lines_;
  boost::circular_buffer<std::vector<ReferenceLine>> reference_line_history_;
  std::future<void> task_future_;

};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_REFERENCE_GENERATOR_REFERENCE_GENERATOR_HPP_
