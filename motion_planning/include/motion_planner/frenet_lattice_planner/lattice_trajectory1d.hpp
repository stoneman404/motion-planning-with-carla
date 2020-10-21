
#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNING_INCLUDE_MOTION_PLANNER_FRENET_LATTICE_PLANNER_LATTICE_TRAJECTORY1D_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNING_INCLUDE_MOTION_PLANNER_FRENET_LATTICE_PLANNER_LATTICE_TRAJECTORY1D_HPP_
#include "polynomial.hpp"
#include <memory>
namespace planning {
class LatticeTrajectory1d : public Polynomial {
 public:
  explicit LatticeTrajectory1d(std::shared_ptr<Polynomial> ptr_trajectory1d);
  ~LatticeTrajectory1d() override = default;
  double Evaluate(size_t order, double param) const override;
  double ParamLength() const override;
  size_t Order() const override;;
  double Coef(size_t order) const override;
 private:
  std::shared_ptr<Polynomial> ptr_trajectory1d_;

};

}
#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNING_INCLUDE_MOTION_PLANNER_FRENET_LATTICE_PLANNER_LATTICE_TRAJECTORY1D_HPP_
