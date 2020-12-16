#include "reference_line/reference_point.hpp"

#include <utility>
namespace planning {

ReferencePoint::ReferencePoint(
    Eigen::Vector2d xy,
    const double &heading, const double &kappa,
    const double &dkappa, const double &ddkappa)
    : xy_(std::move(xy)), theta_(heading),
      kappa_(kappa), dkappa_(dkappa),
      ddkappa_(ddkappa) {}

ReferencePoint::ReferencePoint(const double &x,
                               const double &y,
                               const double &heading,
                               const double &kappa,
                               const double &dkappa,
                               const double &ddkappa)
    : xy_({x, y}), theta_(heading),
      kappa_(kappa),
      dkappa_(dkappa),
      ddkappa_(ddkappa) {}

ReferencePoint::ReferencePoint(const planning::ReferencePoint &other) {
  this->xy_ = other.xy();
  this->theta_ = other.theta();
  this->kappa_ = other.kappa();
  this->dkappa_ = other.dkappa();
  this->ddkappa_ = other.ddkappa();
}
void ReferencePoint::set_xy(const double &x, const double &y) {
  this->xy_[0] = x;
  this->xy_[1] = y;
}
void ReferencePoint::set_xy(const Eigen::Vector2d &xy) { this->xy_ = xy; }
void ReferencePoint::set_theta(const double &heading) { this->theta_ = heading; }
void ReferencePoint::set_kappa(const double &kappa) { this->kappa_ = kappa; }
void ReferencePoint::set_dkappa(const double &dkappa) { this->dkappa_ = dkappa; }
void ReferencePoint::set_ddkappa(const double &ddkappa) { this->ddkappa_ = ddkappa; }
const Eigen::Vector2d &ReferencePoint::xy() const { return xy_; }
const double &ReferencePoint::x() const { return xy_[0]; }
const double &ReferencePoint::y() const { return xy_[1]; }
const double &ReferencePoint::theta() const { return theta_; }
const double &ReferencePoint::kappa() const { return kappa_; }
const double &ReferencePoint::dkappa() const { return dkappa_; }
const double &ReferencePoint::ddkappa() const { return ddkappa_; }

}