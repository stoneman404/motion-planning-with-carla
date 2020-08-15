

#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONTEXT_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONTEXT_HPP_
#include "planning_context.hpp"
#include <planning_srvs/Route.h>
#include <list>

namespace planning{

struct LocalGoal{

  int lane_id = -1;
  bool has_stop_point = false;
  double cruise_speed = 0.0;
};


class PlanningContext{
public:
    static PlanningContext& Instance();
    const LocalGoal& local_goal() const {return local_goal_;}
    LocalGoal& multable_local_goal() { return local_goal_;}


private:
    std::list<planning_srvs::RouteResponse> route_list_;
    LocalGoal local_goal_;

private:

    PlanningContext() = default;
    PlanningContext(const PlanningContext& other);
    PlanningContext& operator=(const PlanningContext& other);
    ~PlanningContext() = default;
};

}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNING_CONTEXT_HPP_
