#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_
// this is a simpler version of behavior planner,

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <planning_srvs/Route.h>

#include <planning_msgs/TrajectoryPoint.h>
#include <Eigen/Core>
#include <vector>
#include "state.hpp"
#include "vehicle_state/vehicle_state.hpp"
#include "reference_line/reference_line.hpp"
#include "planning_context.hpp"

namespace planning {
class State;

enum class DecisionType : uint32_t {
  kFollowLane = 1u,
  kEmergencyStop = 2u,
  kChangeLeft = 3u,
  kChangeRight = 4u,
  kStop = 5u
};

struct ManeuverGoal {
  ManeuverGoal() = default;
  ~ManeuverGoal() = default;
  ManeuverGoal(const int _lane_id, const bool _has_stop_point,
               const double _target_s, const double _target_speed,
               const DecisionType _decision_type) :
      lane_id(_lane_id),
      has_stop_point(_has_stop_point),
      target_s(_target_s),
      target_speed(_target_speed),
      decision_type(_decision_type){}
  int lane_id = -1;
  bool has_stop_point = false;
  double target_s = 0.0;
  double target_speed = 0.0;
  DecisionType decision_type= DecisionType::kFollowLane;
};



class ManeuverPlanner {
 public:
  /**
   *
   * @param nh
   */
  explicit ManeuverPlanner(const ros::NodeHandle &nh);

  /**
   *
   */
  ~ManeuverPlanner();

  void InitPlanner();

  /**
   *
   * @return
   */
  bool Process(const planning_msgs::TrajectoryPoint &init_trajectory_point);

  /**
   *
   * @return
   */
  int GetLaneId() const { return current_lane_id_; }

  /**
   *
   * @param start
   * @param destination
   * @param response
   * @return
   */
  bool ReRoute(const geometry_msgs::Pose &start,
               const geometry_msgs::Pose &destination,
               planning_srvs::RouteResponse &response);

  /**
   *
   * @param reference_lines_list
   * @return
   */
  bool UpdateReferenceLine(
      std::list<std::shared_ptr<ReferenceLine>> *const reference_lines_list) const;

  /**
   *
   * @return
   */
  const planning_msgs::TrajectoryPoint &init_trajectory_point() const;

  void SetManeuverGoal(const ManeuverGoal& maneuver_goal);

 private:

  /**
   *
   * @param ego_pose
   * @param way_points
   * @return
   */
  int GetNearestIndex(const geometry_msgs::Pose &ego_pose,
                      const std::vector<planning_msgs::WayPoint> &way_points) const;

  /**
   *
   * @param matched_index
   * @param backward_distance
   * @param way_points
   * @return
   */
  int GetStartIndex(const int matched_index,
                    double backward_distance,
                    const std::vector<planning_msgs::WayPoint> &way_points) const;

  /**
   *
   * @param matched_index
   * @param forward_distance
   * @param way_points
   * @return
   */
  static int GetEndIndex(const int matched_index,
                  double forward_distance,
                  const std::vector<planning_msgs::WayPoint> &way_points) ;

  /**
   *
   * @param start_index
   * @param end_index
   * @param way_points
   * @return
   */
  static std::vector<planning_msgs::WayPoint>
  GetWayPointsFromStartToEndIndex(const int start_index,
                                  const int end_index,
                                  const std::vector<planning_msgs::WayPoint> &way_points) ;

 private:
  ManeuverGoal maneuver_goal_;
  ros::NodeHandle nh_;
  planning_msgs::TrajectoryPoint init_trajectory_point_;
  ros::ServiceClient route_service_client_;
  std::unique_ptr<State> current_state_;
  int current_lane_id_{};
  std::list<std::shared_ptr<ReferenceLine>> reference_line_;

};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_
