#include "frenet_lattice_planner/lattice_trajectory1d.hpp"

#include <utility>
namespace planning {
LatticeTrajectory1d::LatticeTrajectory1d(std::shared_ptr<Polynomial> ptr_trajectory1d)
    : ptr_trajectory1d_(std::move(ptr_trajectory1d)) {
}

double LatticeTrajectory1d::ParamLength() const {
  return ptr_trajectory1d_->ParamLength();
}
double LatticeTrajectory1d::Evaluate(size_t order, double param) const {
  double param_length = ptr_trajectory1d_->ParamLength();
  if (param < param_length) {
    return ptr_trajectory1d_->Evaluate(order, param);
  }
  double p = ptr_trajectory1d_->Evaluate(0, param_length);
  double v = ptr_trajectory1d_->Evaluate(1, param_length);
  double a = ptr_trajectory1d_->Evaluate(2, param_length);
  double t = param - param_length;
  switch (order) {
    case 0:return p + v * t + 0.5 * a * t * t;
    case 1:return v + a * t;
    case 2:return a;
    default:return 0.0;
  }
}
size_t LatticeTrajectory1d::Order() const { return ptr_trajectory1d_->Order(); }
double LatticeTrajectory1d::Coef(size_t order) const { return ptr_trajectory1d_->Coef(order); }

}
