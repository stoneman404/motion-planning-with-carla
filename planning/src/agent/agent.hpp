#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_AGENT_AGENT_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_AGENT_AGENT_HPP_

#include <ros/ros.h>
#include <unordered_map>
#include <polygon/box2d.hpp>
#include <reference_line/reference_line.hpp>
#include <vehicle_state/vehicle_state.hpp>
#include <obstacle_manager/obstacle.hpp>
#include "behaviour.hpp"
#include "vehicle_state/kinodynamic_state.hpp"
namespace planning {

/**
 * Agent class represents the ego vehicle and other vehicle,
 * other traffic participants like pedestrians and traffic sign/light
 * is not considered in this class
 */
class Agent {
 public:

  Agent();

  /**
   * the ego agent
   * @param vehicle_state
   */
  explicit Agent(const vehicle_state::VehicleState &vehicle_state);

  /**
   * the other agent
   * @param obstacle
   */
  explicit Agent(const Obstacle &obstacle);

  ~Agent() = default;

  int id() const;;

  const planning_msgs::WayPoint &way_point() const;

  const common::Box2d &bounding_box() const;

  const vehicle_state::KinoDynamicState &state() const;

  bool is_valid() const;
  bool is_host() const;

  const std::shared_ptr<ReferenceLine> &current_ref_lane() const;

  const std::shared_ptr<ReferenceLine> &target_ref_lane() const;

  const LateralBehaviour &most_likely_behaviour() const;

  void set_current_ref_lane(const std::shared_ptr<ReferenceLine> &ref_line);

  void set_target_ref_lane(const std::shared_ptr<ReferenceLine> &ref_lane);

  void set_trajectory(const planning_msgs::Trajectory &trajectory);
  void set_most_likely_behaviour(const LateralBehaviour &lateral_behaviour);

  bool PredictAgentBehaviour();

 protected:
  static void StateToPathPoint(const vehicle_state::KinoDynamicState &state, planning_msgs::PathPoint &path_point);

 protected:
  int id_{}; // the agent id
  bool is_host_{true};
  bool is_valid_{false};
  // the bounding box of the agent
  common::Box2d bounding_box_{};
  planning_msgs::WayPoint way_point_;
  vehicle_state::KinoDynamicState state_{};
  LateralBehaviour max_lat_behaviour_{LateralBehaviour::kUndefined};
  ProbDistributeOfLatBehaviour probs_lat_behaviour_;
  std::shared_ptr<ReferenceLine> current_ref_lane_;
  std::shared_ptr<ReferenceLine> target_ref_lane_;
  bool has_trajectory_{false};
  planning_msgs::Trajectory trajectory_;
};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_AGENT_AGENT_HPP_
