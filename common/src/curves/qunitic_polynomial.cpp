#include <array>
#include <ros/assert.h>
#include "curves/quintic_polynomial.hpp"

namespace common {

QuinticPolynomial::QuinticPolynomial(
    const std::array<double, 3> &start, const std::array<double, 3> &end,
    const double param)
    : QuinticPolynomial(start[0], start[1], start[2], end[0], end[1],
                        end[2], param) {}

QuinticPolynomial::QuinticPolynomial(
    double x0, double dx0, double ddx0, double x1,
    double dx1, double ddx1, double param) {
  ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param);
  start_condition_[0] = x0;
  start_condition_[1] = dx0;
  start_condition_[2] = ddx0;
  end_condition_[0] = x1;
  end_condition_[1] = dx1;
  end_condition_[2] = ddx1;
  param_ = param;
}

QuinticPolynomial::QuinticPolynomial(
    const QuinticPolynomial &other) : Polynomial(other) {
  param_ = other.param_;
  coef_ = other.coef_;
}

double QuinticPolynomial::Evaluate(size_t order,
                                   double p) const {
  switch (order) {
    case 0: {
      return ((((coef_[5] * p + coef_[4]) * p + coef_[3]) * p + coef_[2]) * p +
          coef_[1]) *
          p +
          coef_[0];
    }
    case 1: {
      return (((5.0 * coef_[5] * p + 4.0 * coef_[4]) * p + 3.0 * coef_[3]) * p +
          2.0 * coef_[2]) *
          p +
          coef_[1];
    }
    case 2: {
      return (((20.0 * coef_[5] * p + 12.0 * coef_[4]) * p) + 6.0 * coef_[3]) *
          p +
          2.0 * coef_[2];
    }
    case 3: {
      return (60.0 * coef_[5] * p + 24.0 * coef_[4]) * p + 6.0 * coef_[3];
    }
    case 4: {
      return 120.0 * coef_[5] * p + 24.0 * coef_[4];
    }
    case 5: {
      return 120.0 * coef_[5];
    }
    default:return 0.0;
  }
}

void QuinticPolynomial::SetParam(double x0, double dx0,
                                 double ddx0, double x1,
                                 double dx1, double ddx1,
                                 double param) {
  ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param);
  param_ = param;
}

void QuinticPolynomial::ComputeCoefficients(
    double x0, double dx0, double ddx0, double x1,
    double dx1, double ddx1, double p) {
  assert(p > 0.0);

  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = ddx0 / 2.0;

  const double p2 = p * p;
  const double p3 = p * p2;

  // the direct analytical method is at least 6 times faster than using matrix
  // inversion.
  const double c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3;
  const double c1 = (dx1 - ddx0 * p - dx0) / p2;
  const double c2 = (ddx1 - ddx0) / p;

  coef_[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
  coef_[4] = (-15.0 * c0 + 7.0 * c1 - c2) / p;
  coef_[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
}

double QuinticPolynomial::Coef(const size_t order) const {
  assert(6 > order);
  return coef_[order];
}
}