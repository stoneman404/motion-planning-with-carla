#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_VEHICLE_STATE_INCLUDE_VEHICLE_STATE_KINODYNAMIC_STATE_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_VEHICLE_STATE_INCLUDE_VEHICLE_STATE_KINODYNAMIC_STATE_HPP_
#include <Eigen/Core>
#include <planning_msgs/TrajectoryPoint.h>
namespace vehicle_state {

// bicycle vehicle model, center in rear axle center
// x-->front , y-->left, z-->up
struct KinoDynamicState {
 public:
  KinoDynamicState() = default;
  ~KinoDynamicState() = default;
  KinoDynamicState(double x,
                   double y,
                   double z,
                   double theta,
                   double kappa,
                   double v,
                   double a,
                   double centripental_acc,
                   double steer = 0.0);
  void ShiftState(const Eigen::Vector2d &shift_vec);
  KinoDynamicState GetNextStateAfterTime(double t) const;

  planning_msgs::TrajectoryPoint ToTrajectoryPoint(double relative_time = 0.0) ;

  double x;
  double y;
  double z;
  double theta;
  double kappa;
  double v;
  double a;
  double centripental_acc;
  double steer;
};

}

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_VEHICLE_STATE_INCLUDE_VEHICLE_STATE_KINODYNAMIC_STATE_HPP_
