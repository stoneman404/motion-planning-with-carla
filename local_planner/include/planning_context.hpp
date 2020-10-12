#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONTEXT_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONTEXT_HPP_

#include <list>
#include <utility>
#include <planning_srvs/Route.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <carla_msgs/CarlaTrafficLightStatusList.h>
#include <carla_waypoint_types/CarlaWaypoint.h>
#include "vehicle_state/vehicle_state.hpp"
#include "reference_line/reference_line.hpp"

namespace planning {

enum class DecisionType : uint32_t {
  kFollowLane = 1u,
  kEmergencyStop = 2u,
  kChangeLeft = 3u,
  kChangeRight = 4u,
  kStopAtDestination = 5u,
  kStopAtTrafficSign = 6u
};

struct ManeuverInfo {
  ManeuverInfo() = default;
  ~ManeuverInfo() = default;
  int lane_id = 0;
  bool has_stop_point = false;
  union {
    double target_s;
    double target_speed;
  } maneuver_target{};
  std::shared_ptr<ReferenceLine> ptr_ref_line = nullptr;
};

struct ManeuverGoal {
  ManeuverGoal() = default;
  ~ManeuverGoal() = default;

  ManeuverGoal(std::list<ManeuverInfo> _maneuver_infos,
               DecisionType _decision_type) :
      maneuver_infos(std::move(_maneuver_infos)),
      decision_type(_decision_type) {}
  std::list<ManeuverInfo> maneuver_infos;
  DecisionType decision_type = DecisionType::kFollowLane;
};

class PlanningContext {
 public:

  static PlanningContext &Instance();
  /**
   *
   * @return
   */
  std::list<std::shared_ptr<ReferenceLine>> &mutable_reference_lines();
  /**
   *
   * @return
   */
  std::list<planning_srvs::RouteResponse> &mutable_route_infos();

  /**
   *
   * @param goal_pose
   */
  void UpdateGlobalGoalPose(const geometry_msgs::PoseStamped &goal_pose);
  /**
   *
   * @param init_pose
   */
  void UpdateGlobalInitPose(const geometry_msgs::PoseWithCovarianceStamped &init_pose);

  // getter
  /**
   *
   * @return
   */
  const geometry_msgs::PoseWithCovarianceStamped &global_init_pose() const;

  /**
   *
   * @return
   */
  const geometry_msgs::PoseStamped &global_goal_pose() const;

  /**
   *
   * @return
   */
  const std::list<planning_srvs::RouteResponse> &route_infos() const;

  /**
   *
   * @return
   */
  const std::list<std::shared_ptr<ReferenceLine>> &reference_lines() const;

 private:

  std::list<planning_srvs::RouteResponse> route_infos_{};
  // the reference line info
  std::list<std::shared_ptr<ReferenceLine>> reference_lines_{};
  geometry_msgs::PoseStamped global_goal_pose_;
  geometry_msgs::PoseWithCovarianceStamped global_init_pose_;

 private:
  PlanningContext() = default;
  PlanningContext(const PlanningContext &other);
  PlanningContext &operator=(const PlanningContext &other);
  ~PlanningContext() = default;

};

}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONTEXT_HPP_
