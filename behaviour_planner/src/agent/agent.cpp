#include <tf/transform_datatypes.h>
#include "agent.hpp"
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

bool Agent::PredictAgentBehaviour() {
  if (agent_type_ != AgentType::VEHICLE) {
    probs_lat_behaviour_.SetEntry(LateralBehaviour::UNDEFINED, 1.0);
  } else {
    // naive prediction agent's behaviour
    common::FrenetFramePoint frame_point;
    planning_msgs::PathPoint path_point;
    StateToPathPoint(state_, path_point);
    auto frenet_point = current_ref_lane_->GetFrenetFramePoint(path_point);
    double prob_lcl = 0.0;
    double prob_lcr = 0.0;
    double prob_lk = 0.0;
    constexpr double kLatDistanceThreshold = 0.4;
    constexpr double kLatVelThreshold = 0.35; // with respect to s
    if (frenet_point.l > kLatDistanceThreshold && frenet_point.dl > kLatVelThreshold
        && current_ref_lane_->CanChangeLeft(frenet_point.s)) {
      prob_lcl = 1.0;
    } else if (frenet_point.l < -kLatDistanceThreshold && frenet_point.dl < -kLatVelThreshold
        && current_ref_lane_->CanChangeRight(frenet_point.s)) {
      prob_lcr = 1.0;
    } else {
      prob_lk = 1.0;
    }
    probs_lat_behaviour_.SetEntry(LateralBehaviour::LANE_CHANGE_LEFT, prob_lcl);
    probs_lat_behaviour_.SetEntry(LateralBehaviour::LANE_KEEPING, prob_lk);
    probs_lat_behaviour_.SetEntry(LateralBehaviour::LANE_CHANGE_RIGHT, prob_lcr);
    probs_lat_behaviour_.GetMaxProbBehaviour(max_lat_behaviour_);
  }
  return true;
}

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

bool Agent::UpdateAgentStateUsingTrajectoryPoint(planning_msgs::TrajectoryPoint &trajectory_point) {
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

}

