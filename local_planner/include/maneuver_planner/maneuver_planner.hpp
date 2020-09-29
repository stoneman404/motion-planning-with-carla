#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_
// this is a simpler version of behavior planner,

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <planning_srvs/Route.h>
#include <planning_msgs/TrajectoryPoint.h>
#include <planning_msgs/Trajectory.h>
#include <Eigen/Core>
#include <vector>
#include "state.hpp"
#include "vehicle_state/vehicle_state.hpp"
#include "reference_line/reference_line.hpp"
#include "planning_context.hpp"
#include "planner/trajectory_planner.hpp"

namespace planning {

class State;
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

  /**
   *
   */
  void InitPlanner();

  /**
   *
   * @return
   */
  bool Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
               planning_msgs::Trajectory::Ptr pub_trajectory);

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
  static bool UpdateReferenceLine(const std::list<planning_srvs::RouteResponse> &route_list,
                                  std::list<std::shared_ptr<ReferenceLine>> *const reference_lines_list);

  /**
   *
   * @return
   */
  const planning_msgs::TrajectoryPoint &init_trajectory_point() const;

  /**
   *
   * @param maneuver_goal
   */
  void SetManeuverGoal(const ManeuverGoal &maneuver_goal);

  const ManeuverGoal &maneuver_goal() const;

  /**
   *
   * @return
   */
  bool NeedReRoute() const;

 private:

  /**
   *
   * @param ego_pose
   * @param way_points
   * @return
   */
  static int GetNearestIndex(const geometry_msgs::Pose &ego_pose,
                             const std::vector<planning_msgs::WayPoint> &way_points);

  /**
   *
   * @param matched_index
   * @param backward_distance
   * @param way_points
   * @return
   */
  static int GetStartIndex(const int matched_index,
                           double backward_distance,
                           const std::vector<planning_msgs::WayPoint> &way_points);

  /**
   *
   * @param matched_index
   * @param forward_distance
   * @param way_points
   * @return
   */
  static int GetEndIndex(const int matched_index,
                         double forward_distance,
                         const std::vector<planning_msgs::WayPoint> &way_points);

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
                                  const std::vector<planning_msgs::WayPoint> &way_points);

 private:
  ManeuverGoal maneuver_goal_;
  ros::NodeHandle nh_;
  planning_msgs::TrajectoryPoint init_trajectory_point_;
  ros::ServiceClient route_service_client_;
  std::unique_ptr<State> current_state_;
  int current_lane_id_{};

};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_
