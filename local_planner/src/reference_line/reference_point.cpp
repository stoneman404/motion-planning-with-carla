#include "reference_line/reference_point.hpp"
namespace planning {

ReferencePoint::ReferencePoint(
    const Eigen::Vector2d &xy,
    const double &heading, const double &kappa,
    const double &dkappa, const double &ddkappa)
    : xy_(xy), heading_(heading),
      kappa_(kappa), dkappa_(dkappa),
      ddkappa_(ddkappa) {}

ReferencePoint::ReferencePoint(const double &x,
                               const double &y,
                               const double &heading,
                               const double &kappa,
                               const double &dkappa,
                               const double &ddkappa)
    : xy_({x, y}), heading_(heading),
      kappa_(kappa),
      dkappa_(dkappa),
      ddkappa_(ddkappa) {}

ReferencePoint::ReferencePoint(const planning::ReferencePoint &other) {
  this->xy_ = other.xy();
  this->heading_ = other.heading();
  this->kappa_ = other.kappa();
  this->dkappa_ = other.dkappa();
  this->ddkappa_ = other.ddkappa();
}

}