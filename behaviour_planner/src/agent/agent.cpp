#include <tf/transform_datatypes.h>
#include "agent.hpp"
#include "math/coordinate_transformer.hpp"
#include <boost/math/distributions/normal.hpp>
namespace planning {

Agent::Agent(const derived_object_msgs::Object &object)
    : id_(object.id),
      is_host_(false),
      is_valid_(true),
      bounding_box_(common::Box2d({object.pose.position.x,
                                   object.pose.position.y},
                                  tf::getYaw(object.pose.orientation),
                                  object.shape.dimensions[object.shape.BOX_X],
                                  object.shape.dimensions[object.shape.BOX_Y])),
      length_(object.shape.dimensions[object.shape.BOX_X]),
      width_(object.shape.dimensions[object.shape.BOX_Y]),
      max_lat_behaviour_(LateralBehaviour::UNDEFINED),
      current_ref_lane_(nullptr),
      target_ref_lane_(nullptr),
      trajectory_(planning_msgs::Trajectory()) {

//  speed_buffer_.set_capacity(30);
  double x = bounding_box_.center_x();
  double y = bounding_box_.center_y();
  double z = object.pose.position.z;
  double theta = bounding_box_.heading();
  double v = std::hypot(object.twist.linear.x, object.twist.linear.y);
  double a = object.accel.linear.x * std::cos(theta) + object.accel.linear.y * std::sin(theta);
  double centripental_acc = object.accel.linear.y * std::cos(theta) - object.accel.linear.x * std::sin(theta);
  if (std::fabs(a) < 0.4 && std::fabs(v) < 0.1) {
    is_static_ = true;
  }
  double kappa = 0.0;
  if (v > 1e-3) {
    kappa = centripental_acc / (v * v);
  }
  state_ = vehicle_state::KinoDynamicState(x, y, z, theta, kappa, v, a, centripental_acc);
  RetriveAgentType(object);
}

Agent::Agent(const carla_msgs::CarlaTrafficLightStatus &traffic_light_status,
             const carla_msgs::CarlaTrafficLightInfo &traffic_light_info)
    : id_(traffic_light_info.id),
      is_host_(false),
      is_valid_(true),
      max_lat_behaviour_(LateralBehaviour::UNDEFINED),
      current_ref_lane_(nullptr),
      target_ref_lane_(nullptr),
      is_static_(true),
      trajectory_(planning_msgs::Trajectory()),
      agent_type_(AgentType::TRAFFIC_LIGHT) {
  double box_length = traffic_light_info.trigger_volume.size.x;
  double box_width = traffic_light_info.trigger_volume.size.y;
  length_ = box_length;
  width_ = box_width;
  Eigen::Vector2d center{traffic_light_info.transform.position.x + traffic_light_info.trigger_volume.center.x,
                         traffic_light_info.transform.position.y + traffic_light_info.trigger_volume.center.y};
  double heading = tf::getYaw(traffic_light_info.transform.orientation);
  bounding_box_ = common::Box2d(center, heading, box_length, box_width);
}

Agent::Agent(const vehicle_state::VehicleState &vehicle_state) : id_(vehicle_state.id()),
                                                                 is_host_(true),
                                                                 is_valid_(true),
                                                                 bounding_box_(vehicle_state.GetEgoBox()),
                                                                 length_(bounding_box_.length()),
                                                                 width_(bounding_box_.width()),
                                                                 state_(vehicle_state.GetKinoDynamicVehicleState()),
                                                                 max_lat_behaviour_(LateralBehaviour::UNDEFINED),
                                                                 current_ref_lane_(nullptr),
                                                                 target_ref_lane_(nullptr),
                                                                 trajectory_(planning_msgs::Trajectory()),
                                                                 agent_type_(AgentType::VEHICLE) {
  if (std::fabs(state_.v) < 0.1 && std::fabs(state_.a) < 0.4) {
    is_static_ = true;
  }

}

Agent::Agent() : id_(-1),
                 is_host_(false),
                 is_valid_(false),
                 bounding_box_(common::Box2d()),
                 length_(0.0),
                 width_(0.0),
                 state_(vehicle_state::KinoDynamicState()),
                 max_lat_behaviour_(LateralBehaviour::UNDEFINED),
                 current_ref_lane_(nullptr),
                 target_ref_lane_(nullptr),
                 is_static_(false),
                 trajectory_(planning_msgs::Trajectory()),
                 agent_type_(AgentType::UNKNOWN) {
}

void Agent::RetriveAgentType(const derived_object_msgs::Object &object) {
  switch (object.classification) {
    case derived_object_msgs::Object::CLASSIFICATION_CAR:
    case derived_object_msgs::Object::CLASSIFICATION_MOTORCYCLE:
    case derived_object_msgs::Object::CLASSIFICATION_OTHER_VEHICLE:
    case derived_object_msgs::Object::CLASSIFICATION_TRUCK: {
      agent_type_ = AgentType::VEHICLE;
      break;
    }
    case derived_object_msgs::Object::CLASSIFICATION_PEDESTRIAN: {
      agent_type_ = AgentType::PEDESTRIAN;
      break;
    }
    case derived_object_msgs::Object::CLASSIFICATION_BIKE: {
      agent_type_ = AgentType::BIKE;
      break;
    }
    case derived_object_msgs::Object::CLASSIFICATION_BARRIER:
    case derived_object_msgs::Object::CLASSIFICATION_SIGN:
    case derived_object_msgs::Object::CLASSIFICATION_UNKNOWN:
    case derived_object_msgs::Object::CLASSIFICATION_UNKNOWN_BIG:
    case derived_object_msgs::Object::CLASSIFICATION_UNKNOWN_MEDIUM:
    case derived_object_msgs::Object::CLASSIFICATION_UNKNOWN_SMALL:
    default: {
      agent_type_ = AgentType::UNKNOWN;
      break;
    }
  }
}

const common::Box2d &Agent::bounding_box() const {
  return bounding_box_;
}

int Agent::id() const { return id_; }

const vehicle_state::KinoDynamicState &Agent::state() const {
  return state_;
}

bool Agent::is_valid() const {
  return is_valid_;
}

bool Agent::is_host() const {
  return is_host_;
}

const std::shared_ptr<ReferenceLine> &Agent::current_ref_lane() const {
  return current_ref_lane_;
}

const std::shared_ptr<ReferenceLine> &Agent::target_ref_lane() const {
  return target_ref_lane_;
}

const LateralBehaviour &Agent::most_likely_behaviour() const {
  return max_lat_behaviour_;
}

void Agent::set_current_ref_lane(const std::shared_ptr<ReferenceLine> &ref_line) {
  this->current_ref_lane_ = ref_line;
}

void Agent::set_target_ref_lane(const std::shared_ptr<ReferenceLine> &ref_lane) {
  this->target_ref_lane_ = ref_lane;
}

void Agent::set_trajectory(const planning_msgs::Trajectory &trajectory) {
  this->trajectory_ = trajectory;
}

void Agent::set_most_likely_behaviour(const LateralBehaviour &lateral_behaviour) {
  max_lat_behaviour_ = lateral_behaviour;
}

//bool Agent::PredictAgentBehaviour() {
//  if (agent_type_ != AgentType::VEHICLE) {
//    probs_lat_behaviour_.SetEntry(LateralBehaviour::UNDEFINED, 1.0);
//  } else {
//    // naive prediction agent's behaviour
//    common::FrenetFramePoint frame_point;
//    planning_msgs::PathPoint path_point;
//    StateToPathPoint(state_, path_point);
//    auto frenet_point = current_ref_lane_->GetFrenetFramePoint(path_point);
//    double prob_lcl = 0.0;
//    double prob_lcr = 0.0;
//    double prob_lk = 0.0;
//    constexpr double kLatDistanceThreshold = 0.4;
//    constexpr double kLatVelThreshold = 0.35; // with respect to s
//    if (frenet_point.l > kLatDistanceThreshold && frenet_point.dl > kLatVelThreshold
//        && current_ref_lane_->CanChangeLeft(frenet_point.s)) {
//      prob_lcl = 1.0;
//    } else if (frenet_point.l < -kLatDistanceThreshold && frenet_point.dl < -kLatVelThreshold
//        && current_ref_lane_->CanChangeRight(frenet_point.s)) {
//      prob_lcr = 1.0;
//    } else {
//      prob_lk = 1.0;
//    }
//    probs_lat_behaviour_.SetEntry(LateralBehaviour::LANE_CHANGE_LEFT, prob_lcl);
//    probs_lat_behaviour_.SetEntry(LateralBehaviour::LANE_KEEPING, prob_lk);
//    probs_lat_behaviour_.SetEntry(LateralBehaviour::LANE_CHANGE_RIGHT, prob_lcr);
//    probs_lat_behaviour_.GetMaxProbBehaviour(max_lat_behaviour_);
//  }
//  return true;
//}

void Agent::StateToPathPoint(const vehicle_state::KinoDynamicState &state, planning_msgs::PathPoint &path_point) {
  path_point.x = state.x;
  path_point.y = state.y;
  path_point.theta = state.theta;
  path_point.kappa = state.kappa;
  path_point.dkappa = 0.0;
}

const AgentType &Agent::agent_type() const {
  return agent_type_;
}

bool Agent::MoveAgentToPoint(planning_msgs::TrajectoryPoint &trajectory_point) {
  state_.x = trajectory_point.path_point.x;
  state_.y = trajectory_point.path_point.y;
  state_.z = 0.0;
  state_.theta = trajectory_point.path_point.theta;
  state_.kappa = trajectory_point.path_point.kappa;
  state_.v = trajectory_point.vel;
  state_.a = trajectory_point.acc;
  state_.centripental_acc = state_.v * state_.v * state_.kappa;
  Eigen::Vector2d center{state_.x, state_.y};
  bounding_box_ = common::Box2d(center, state_.theta, length_, width_);
  return true;
}

bool Agent::is_static() const {
  return is_static_;
}

bool Agent::GetKMaxProbBehavioursAndLanes(
    uint32_t k, std::vector<std::pair<LateralBehaviour, std::shared_ptr<ReferenceLine>>> &behaviour_with_lanes) {
  if (k < 1) {
    return false;
  }
  return probs_lat_behaviour_.GetKthMaxProbBehavioursAndLanes(k, behaviour_with_lanes);
}

bool Agent::PredictAgentBehaviour(const std::vector<std::shared_ptr<ReferenceLine>> &potential_lanes,
                                  double max_lon_acc,
                                  double min_lon_acc,
                                  double sim_step,
                                  double max_lat_acc,
                                  double default_desired_vel) {

  // there is no potential lanes for agent, the behaviour is undefined.
  if (potential_lanes.empty()) {
    probs_lat_behaviour_.SetEntry(LateralBehaviour::UNDEFINED, 1.0, std::shared_ptr<ReferenceLine>());
    return true;
  }
  // make decision the potential lane as well as the implied behaviours
  vehicle_state::KinoDynamicState naive_predicted_state{};
  if (!PredictStateFromCurrentState(state_, naive_predicted_state, sim_step)) {
    probs_lat_behaviour_.SetEntry(LateralBehaviour::UNDEFINED, 1.0, std::shared_ptr<ReferenceLine>());
  }
  std::vector<std::array<double, 2>> feature_vectors;
  feature_vectors.reserve(potential_lanes.size());
  double evidence = 0;
  boost::math::normal_distribution<> speed_feature_distribution(0.0, 2.0);
  boost::math::normal_distribution<> position_feature_distribution(0.0, std::sqrt(6.0));
  std::vector<std::pair<std::shared_ptr<ReferenceLine>, LateralBehaviour>> valid_lanes;

  //ref : Decision Making for Autonomous Driving considering Interaction and
  //Uncertain Prediction of Surrounding Vehicles eqn.(14-16)
  for (const auto &lane : potential_lanes) {
    auto ref_point = lane->GetReferencePoint(state_.x, state_.y);
    Eigen::Vector3d predicted_state;
    if (!PredictStateOnPrefixedLane(lane,
                                    state_,
                                    default_desired_vel,
                                    sim_step,
                                    max_lat_acc,
                                    max_lon_acc,
                                    min_lon_acc,
                                    predicted_state)) {
      continue;
    }

    std::array<double, 2> feature_vec{std::fabs(naive_predicted_state.v - predicted_state[2]),
                                      std::hypot(naive_predicted_state.x - predicted_state[0],
                                                 naive_predicted_state.y - predicted_state[1])};
    evidence += boost::math::pdf(speed_feature_distribution, feature_vec[0])
        * boost::math::pdf(position_feature_distribution, feature_vec[1]);
    feature_vectors.push_back(feature_vec);
    valid_lanes.emplace_back(lane, PredictBehaviourFromRefLane(lane));
  }
  assert(valid_lanes.size() == feature_vectors.size());
  for (size_t i = 0; i < valid_lanes.size(); ++i) {
    double prob = boost::math::pdf(speed_feature_distribution, feature_vectors[i][0])
        * boost::math::pdf(position_feature_distribution, feature_vectors[i][1]) / evidence;
    probs_lat_behaviour_.SetEntry(valid_lanes[i].second, std::max(0.0, std::min(prob, 1.0)), valid_lanes[i].first);
  }
  return true;
}

LateralBehaviour Agent::PredictBehaviourFromRefLane(const std::shared_ptr<ReferenceLine> &lane) const {

  constexpr double kTurnAngleThreshold = 0.195 * M_PI;
  constexpr double kLengthStep = 2.0;
  constexpr double kLateralThreshold = 0.25;
  constexpr double kFrontDistanceThreshold = 10.0;
  const double ref_length = lane->Length();
  // the different lane represents the different behaviour.
  common::SLPoint sl_point;
  if (!lane->XYToSL(state_.x, state_.y, &sl_point)) {
    return LateralBehaviour::UNDEFINED;
  }
  if (sl_point.l > kLateralThreshold) {
    return LateralBehaviour::LANE_CHANGE_RIGHT;
  }
  if (sl_point.l < -kLateralThreshold) {
    return LateralBehaviour::LANE_CHANGE_LEFT;
  }

  auto last_ref_point = lane->GetReferencePoint(0.0);
  if (!lane->HasJunctionInFront(state_.x, state_.y, kFrontDistanceThreshold)) {
    return LateralBehaviour::LANE_KEEPING;
  }
  double s = kLengthStep;
  while (s < ref_length) {
    auto ref_point = lane->GetReferencePoint(s);
    double angle_diff = common::MathUtils::CalcAngleDist(last_ref_point.theta(), ref_point.theta());
    if (angle_diff > kTurnAngleThreshold) {
      return LateralBehaviour::TURN_LEFT;
    }
    if (angle_diff < -kTurnAngleThreshold) {
      return LateralBehaviour::TURN_RIGHT;
    }
    s += kLengthStep;
  }
  return LateralBehaviour::GO_STRAIGHT;
}

bool Agent::PredictStateOnPrefixedLane(const std::shared_ptr<ReferenceLine> &lane,
                                       const vehicle_state::KinoDynamicState &current_state,
                                       double desired_velocity,
                                       double sim_step,
                                       double max_lat_acc,
                                       double max_lon_acc,
                                       double min_lon_acc,
                                       Eigen::Vector3d predict_state) {
  constexpr double kLateralThreshold = 0.25;
  common::SLPoint sl_point;
  if (!lane->XYToSL(current_state.x, current_state.y, &sl_point)) {
    return false;
  }
  auto ref_point = lane->GetReferencePoint(sl_point.s);
  double lateral_approach_ratio = 0.995; // we assume the agent is approaching this lane ...
  if (sl_point.l > kLateralThreshold || sl_point.l < -kLateralThreshold) {
    lateral_approach_ratio = 0.95; // we assume the agent is cutting in this lane ...
  }
  double x = current_state.x;
  double y = current_state.y;
  double v = current_state.v;
  double a = current_state.a;
  double theta = current_state.theta;
  double kappa = current_state.kappa;
  double rs = sl_point.s;
  double rx = ref_point.x();
  double ry = ref_point.y();
  double rtheta = ref_point.theta();
  double rkappa = ref_point.kappa();
  double rdkappa = ref_point.dkappa();
  std::array<double, 3> s_conditions{0, 0, 0};
  std::array<double, 3> d_conditions{0, 0, 0};
  common::CoordinateTransformer::CartesianToFrenet(rs, rx, ry, rtheta, rkappa, rdkappa,
                                                   x, y, v, a, theta, kappa,
                                                   &s_conditions, &d_conditions);
  double actual_desired_vel =
      std::min(max_lat_acc / std::fabs(lane->GetReferencePoint(sl_point.s).kappa()) + 1e-5, desired_velocity);
  double vel_gap = actual_desired_vel - s_conditions[1];
  double a_ref = vel_gap / sim_step;
  double motion_a = std::max(std::min(a_ref, max_lon_acc), min_lon_acc);

  double next_s = s_conditions[0] + s_conditions[1] * sim_step + 0.5 * sim_step * sim_step * motion_a;
  double next_d = d_conditions[0] * lateral_approach_ratio;
  auto next_ref_point = lane->GetReferencePoint(next_s);
  auto next_state = common::CoordinateTransformer::CalcCatesianPoint(next_ref_point.theta(),
                                                                     next_ref_point.x(),
                                                                     next_ref_point.y(), next_d);
  double
      next_ref_v = std::min(max_lat_acc / std::fabs(lane->GetReferencePoint(next_s).kappa()) + 1e-5, desired_velocity);
  predict_state << next_state.x(), next_state.y(), next_ref_v;
  return true;
}

bool Agent::PredictStateFromCurrentState(const vehicle_state::KinoDynamicState &current_state,
                                         vehicle_state::KinoDynamicState &predicted_state,
                                         double sim_step) {
  predicted_state = current_state.GetNextStateAfterTime(sim_step);
  return true;
}

}