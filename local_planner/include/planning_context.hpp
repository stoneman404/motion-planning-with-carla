#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONTEXT_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONTEXT_HPP_
#include <planning_srvs/Route.h>

#include <list>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "planning_context.hpp"
#include "vehicle_state/vehicle_state.hpp"


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
    std::list<planning_srvs::RouteResponse>& mutable_route_infos() {return route_infos_;}
//    void UpdateRouteInfo(const VehicleState &vehicle_state,
//                         const planning_srvs::RouteResponse &route_response);
    void UpdateLocalGoal(int lane_id = 0, bool has_stop_point = false,
                         double stop_s = 0, double cruise_speed = 0);
    void UpdateLocalGoal(const LocalGoal &local_goal);
    void UpdateGlobalGoalPose(const geometry_msgs::PoseStamped& goal_pose) ;
    void UpdateGlobalInitPose(const geometry_msgs::PoseWithCovarianceStamped& init_pose);
    // getter
    const geometry_msgs::PoseWithCovarianceStamped& global_init_pose() const ;
    const geometry_msgs::PoseStamped& global_goal_pose() const;
    const std::list<planning_srvs::RouteResponse> &route_infos() const;
    const std::list<std::shared_ptr<ReferenceLine>> &reference_lines() const;
    const LocalGoal &local_goal() const;


private:

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
