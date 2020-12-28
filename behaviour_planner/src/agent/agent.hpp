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
#include <derived_object_msgs/Object.h>
#include <boost/circular_buffer.hpp>

namespace planning {

enum class AgentType : uint32_t {
  UNKNOWN = 0u,
  VEHICLE = 1u,
  BIKE = 2U,
  PEDESTRIAN = 3U,
  TRAFFIC_LIGHT = 4u
};

/**
 * Agent class represents the ego vehicle and other vehicle,
 */
class Agent {
 public:
  Agent();
  /**
   * @brief: the object agent, e.g. other vehicles, bike, pedestrian, motorcycle
   * @param object
   */
  explicit Agent(const derived_object_msgs::Object &object);
  /**
   * @brief: the ego agent
   * @param vehicle_state
   */
  explicit Agent(const vehicle_state::VehicleState &vehicle_state);

  /**
   * @brief: traffic light agent
   * @param traffic_light_status
   * @param traffic_light_info
   */
  Agent(const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
        const carla_msgs::CarlaTrafficLightInfo &traffic_light_info);

  ~Agent() = default;

  /**
   * @brief: move the agent to the trajectory point
   * @param trajectory_point
   * @return
   */
  bool MoveAgentToPoint(planning_msgs::TrajectoryPoint &trajectory_point);

  /**
   * @brief: the id of this agent
   * @return
   */
  int id() const;;

  /**
   * @brief: the bounding box of this agent
   * @return
   */
  const common::Box2d &bounding_box() const;

  /**
   * @brief: the kino dynamic state of agent: pose, v, a, kappa, etc,
   * @return
   */
  const vehicle_state::KinoDynamicState &state() const;

  /**
   * @brief: is valid agent
   * @return
   */
  bool is_valid() const;

  /**
   * @brief: is host agent
   * @return
   */
  bool is_host() const;

  /**
   * @brief: is static agent
   * @return
   */
  bool is_static() const;

  /**
   * @brief the agent type
   * @return
   */
  const AgentType &agent_type() const;
  const std::shared_ptr<ReferenceLine> &current_ref_lane() const;
  const std::shared_ptr<ReferenceLine> &target_ref_lane() const;

  /**
   * @brief: get the top k's probs and lanes
   * @param k : number of behaviours
   * @param behaviour_with_lanes: behaviour and ref lane
   * @return
   */
  bool GetKMaxProbBehavioursAndLanes(
      uint32_t k, std::vector<std::pair<LateralBehaviour, std::shared_ptr<ReferenceLine>>> &behaviour_with_lanes);

  /**
   * @brief: get the most likely behaviour
   * @return
   */
  const LateralBehaviour &most_likely_behaviour() const;

  /**   setter   **/
  /**
   * @param ref_line
   */
  void set_current_ref_lane(const std::shared_ptr<ReferenceLine> &ref_line);
  void set_target_ref_lane(const std::shared_ptr<ReferenceLine> &ref_lane);
  void set_is_host(bool host_agent) { is_host_ = host_agent; }
  void set_trajectory(const planning_msgs::Trajectory &trajectory);
  void set_most_likely_behaviour(const LateralBehaviour &lateral_behaviour);

  /**
   * @brief: predict the agent's behaviour from potential lanes
   * @param potential_lanes
   * @return
   */
  bool PredictAgentBehaviour(const std::vector<std::shared_ptr<ReferenceLine>> &potential_lanes,
                             double max_lon_acc,
                             double min_lon_acc,
                             double sim_step,
                             double max_lat_acc,
                             double default_desired_vel);

 protected:

  /**
   * @brief: get the agent type from derived_object_msgs
   * @param object
   */
  void RetriveAgentType(const derived_object_msgs::Object &object);

  /**
   * @brief: convert kinodynamic state to path point
   * @param state
   * @param path_point
   */
  static void StateToPathPoint(const vehicle_state::KinoDynamicState &state, planning_msgs::PathPoint &path_point);

  /**
   * @brief: predict agent's behaviour based on ref lane, if the ref lane has junction,
   * and the turn angle from two near ref point exceed the turn angle threshold, we may
   * suppose the agent will turn right or left in junction, otherwise go straight
   * @param lane: the lane.
   * @return
   */
  LateralBehaviour PredictBehaviourFromRefLane(const std::shared_ptr<ReferenceLine> &lane) const;

  /**
   * @brief predict the state on prefixed lane, simulate one step forward to generate the predicted state on lane
   * @param[in] lane: the prefixed lane
   * @param[in] current_state: the current state
   * @param[in] desired_velocity: desired speed, user defined
   * @param[in] sim_step: sim step, typical: 0.25
   * @param[in] max_lat_acc: max lateral acc , w.r.t time[s]
   * @param[in] max_lon_acc: max longitudinal acc, w.r.t. time[s]
   * @param[in] min_lon_acc: min longitudinal acc, w.r.t. time[s]
   * @param[out] predict_state: the predicted state.
   * @return: true if the procedure is successful.
   */
  static bool PredictStateOnPrefixedLane(const std::shared_ptr<ReferenceLine> &lane,
                                         const vehicle_state::KinoDynamicState &current_state,
                                         double desired_velocity,
                                         double sim_step,
                                         double max_lat_acc,
                                         double max_lon_acc,
                                         double min_lon_acc,
                                         Eigen::Vector3d predict_state);

  /**
   * @brief: forward simulate the agent one step, without consider the behaviour.
   * @param[in] current_state: the current state of agent
   * @param[out] predicted_state: the predicted state
   * @param[in] sim_step: sim step
   * @return
   */
  static bool PredictStateFromCurrentState(const vehicle_state::KinoDynamicState &current_state,
                                           vehicle_state::KinoDynamicState &predicted_state,
                                           double sim_step);

 protected:
  int id_{}; // the agent id
  bool is_host_{true};
  bool is_valid_{false};
  // the bounding box of the agent
  common::Box2d bounding_box_{};
  double length_{};
  double width_{};
  vehicle_state::KinoDynamicState state_{};
  LateralBehaviour max_lat_behaviour_{LateralBehaviour::UNDEFINED};
  ProbDistributeOfLatBehaviour probs_lat_behaviour_;
  std::shared_ptr<ReferenceLine> current_ref_lane_;
  std::shared_ptr<ReferenceLine> target_ref_lane_;
  bool is_static_{false};
  planning_msgs::Trajectory trajectory_;
  AgentType agent_type_;
};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_PLANNING_SRC_AGENT_AGENT_HPP_
