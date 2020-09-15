#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONTEXT_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONTEXT_HPP_
#include <planning_srvs/Route.h>

#include <list>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <carla_msgs/CarlaTrafficLightStatusList.h>
#include "planning_context.hpp"
#include "vehicle_state/vehicle_state.hpp"
#include <carla_waypoint_types/CarlaWaypoint.h>
#include "reference_line/reference_line.hpp"

namespace planning {

struct LocalGoal {

  int lane_id = -1;
  bool has_stop_point = false;
  double stop_s = 0.0;
  double cruise_speed = 0.0;
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
     * @param lane_id
     * @param has_stop_point
     * @param stop_s
     * @param cruise_speed
     */
    void UpdateLocalGoal(int lane_id = 0, bool has_stop_point = false,
                         double stop_s = 0, double cruise_speed = 0);
    /**
     *
     * @param local_goal
     */
    void UpdateLocalGoal(const LocalGoal &local_goal);

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

    /**
     *
     * @param traffic_lights
     */
    void UpdateTrafficLights(
        const std::vector<std::pair<carla_msgs::CarlaTrafficLightStatus,
                                    carla_waypoint_types::CarlaWaypoint>> &traffic_lights);
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

    /**
     *
     * @return
     */
    const LocalGoal &local_goal() const;

    /**
     *
     * @return
     */
    const std::vector<std::pair<carla_msgs::CarlaTrafficLightStatus,
                                carla_waypoint_types::CarlaWaypoint>> &TrafficLights() const;

private:

    std::vector<
        std::pair<carla_msgs::CarlaTrafficLightStatus,
                  carla_waypoint_types::CarlaWaypoint>> traffic_lights_;

    std::list<planning_srvs::RouteResponse> route_infos_{};
    LocalGoal local_goal_;
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
