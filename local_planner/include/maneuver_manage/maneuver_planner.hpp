#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_
// this is a simpler version of behavior planner,

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <Eigen/Core>
#include <vector>
#include "state.hpp"
#include "vehicle_state/vehicle_state.hpp"

namespace planning {
class State;
class ManeuverPlanner {
public:
    explicit ManeuverPlanner(const ros::NodeHandle &nh);
    ~ManeuverPlanner() = default;
    
    void InitPlanner();

    /**
     *
     * @return
     */
    bool Process();

    /**
     *
     * @param new_state
     */
    void SetState(State &new_state);

    /**
     *
     * @return
     */
    std::shared_ptr<State> GetState() const { return current_state_; };

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
    bool UpdateReferenceLine(std::list<std::shared_ptr<ReferenceLine>> *const reference_lines_list) const;
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
    int GetEndIndex(const int matched_index,
                    double forward_distance,
                    const std::vector<planning_msgs::WayPoint> &way_points) const;

    /**
     *
     * @param start_index
     * @param end_index
     * @param way_points
     * @return
     */
    std::vector<planning_msgs::WayPoint> GetWayPointsFromStartToEndIndex(const int start_index,
                                                                         const int end_index,
                                                                         const std::vector<planning_msgs::WayPoint> &way_points) const;
private:
    ros::NodeHandle nh_;
    ros::ServiceClient route_service_client_;
    std::shared_ptr<State> current_state_;
    int current_lane_id_;
    std::list<std::shared_ptr<ReferenceLine>> reference_line_;

};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MANEUVER_PLANNER_MANEUVER_PLANNER_HPP_
