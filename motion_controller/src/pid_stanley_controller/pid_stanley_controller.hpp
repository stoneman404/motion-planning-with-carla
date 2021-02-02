
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_PID_STANLEY_CONTROLLER_PID_STANLEY_CONTROLLER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_PID_STANLEY_CONTROLLER_PID_STANLEY_CONTROLLER_HPP_

#include <planning_msgs/Trajectory.h>
#include "control_strategy.hpp"
#include "control_config.hpp"
#include <boost/circular_buffer.hpp>

namespace control {
class PIDStanleyController : public ControlStrategy {
 public:
  PIDStanleyController() = default;
  PIDStanleyController(const ControlConfigs &control_configs, double loop_rate);
  ~PIDStanleyController() override = default;
  bool Execute(double current_time_stamp,
               const vehicle_state::VehicleState &vehicle_state,
               const planning_msgs::Trajectory &trajectory,
               carla_msgs::CarlaEgoVehicleControl &control) override;

 private:
  bool LongitudinalControl(double current_speed, double target_speed, double delta_t, double *throttle);

  bool StanleySteerControl(double max_steer,
                               double wheelbase,
                               const vehicle_state::KinoDynamicState &vehicle_state,
                               const planning_msgs::TrajectoryPoint &target_tp,
                               const planning_msgs::Trajectory &trajectory,
                               double *steer);
  template<class T>
  inline T Clamp(const T &value, const T &lower, const T &upper);

  static inline bool GetMatchedPointByPosition(double x,
                                               double y,
                                               const planning_msgs::Trajectory &trajectory,
                                               planning_msgs::TrajectoryPoint &matched_tp);

  static inline bool GetMatchedPointByAbsoluteTime(double &time_stamp, const planning_msgs::Trajectory& trajectory,
                                                   planning_msgs::TrajectoryPoint& matched_tp);

  static inline bool GetMatchedPointByRelativeTime(double relative_time, const planning_msgs::Trajectory& trajectory,
                                                   planning_msgs::TrajectoryPoint& matched_tp);

  static inline bool GetMatchedPointByS(double s, const planning_msgs::Trajectory& trajectory,
                                        planning_msgs::TrajectoryPoint& matched_tp);




 private:
  ControlConfigs control_configs_;
  double loop_rate_{};
  boost::circular_buffer<double> lat_error_buffer_{};
  boost::circular_buffer<double> lon_error_buffer_{};

};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_PID_STANLEY_CONTROLLER_PID_STANLEY_CONTROLLER_HPP_
