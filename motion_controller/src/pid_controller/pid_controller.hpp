#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_PID_CONTROLLER_PID_CONTROLLER_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_PID_CONTROLLER_PID_CONTROLLER_HPP_
#include <planning_msgs/Trajectory.h>
#include "control_strategy.hpp"
#include "control_config.hpp"
#include <boost/circular_buffer.hpp>

namespace control {
class PIDController : public ControlStrategy {
 public:
  PIDController() = default;
  PIDController(const PIDConfigs &pid_configs, double loop_rate);
  ~PIDController() override = default;
  bool Execute(double current_time_stamp,
               const vehicle_state::VehicleState &vehicle_state,
               const planning_msgs::Trajectory &trajectory,
               carla_msgs::CarlaEgoVehicleControl &control) override;

 private:
  bool LongitudinalControl(double current_speed, double target_speed, double delta_t, double *throttle);
  bool LateralControl(const Eigen::Vector3d &current_pose,
                      const Eigen::Vector3d &target_pose,
                      double delta_t,
                      double *steer);
  template<class T>
  inline T Clamp(const T &value, const T &lower, const T &upper);

  static inline bool GetMatchedPointByPosition(double x,
                                 double y,
                                 const planning_msgs::Trajectory &trajectory,
                                 planning_msgs::TrajectoryPoint &matched_tp);

  static inline bool GetMatchedPointByAbsoluteTime(const ros::Time &time_stamp, const planning_msgs::Trajectory& trajectory,
                                                   planning_msgs::TrajectoryPoint& matched_tp);

  static inline bool GetMatchedPointByRelativeTime(double relative_time, const planning_msgs::Trajectory& trajectory,
                                                   planning_msgs::TrajectoryPoint& matched_tp);


 private:
  PIDConfigs pid_configs_;
  double loop_rate_;
  boost::circular_buffer<double> lat_error_buffer_{};
  boost::circular_buffer<double> lon_error_buffer_{};

};
}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_CONTROLLER_SRC_PID_CONTROLLER_PID_CONTROLLER_HPP_
