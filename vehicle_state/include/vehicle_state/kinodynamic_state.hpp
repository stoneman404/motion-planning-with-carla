#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_VEHICLE_STATE_INCLUDE_VEHICLE_STATE_KINODYNAMIC_STATE_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_VEHICLE_STATE_INCLUDE_VEHICLE_STATE_KINODYNAMIC_STATE_HPP_
namespace vehicle_state{

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
                   double centripental_acc);
  KinoDynamicState GetNextStateAfterTime(double t) const;


  double x_;
  double y_;
  double z_;
  double theta_;
  double kappa_;
  double v_;
  double a_;
  double centripental_acc_;
};

}


#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_VEHICLE_STATE_INCLUDE_VEHICLE_STATE_KINODYNAMIC_STATE_HPP_
