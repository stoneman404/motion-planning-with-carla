#include <array>
#include <ros/assert.h>
#include "curves/quintic_polynomial.hpp"

namespace common {
QuinticPolynomial::QuinticPolynomial(const std::array<double, 3> &start,
                                     const std::array<double, 3> &end,
                                     double param)
    : start_condition_(start),
      end_condition_(end) {
  param_ = param;
  order_ = 5;
  SetUpPolynomial(start[0], start[1], start[2], end[0], end[1], end[2], param);
}

QuinticPolynomial::QuinticPolynomial(double x0,
                                     double dx0,
                                     double ddx0,
                                     double x1,
                                     double dx1,
                                     double ddx1,
                                     double param)
    : start_condition_({x0, dx0, ddx0}),
      end_condition_({x1, dx1, ddx1}) {
  param_ = param;
  order_ = 5;
  SetUpPolynomial(x0, dx0, ddx0, x1, dx1, ddx1, param);
}
QuinticPolynomial::QuinticPolynomial(const QuinticPolynomial &other) {
  order_ = other.order_;
  param_ = other.param_;
  coef_ = other.coef_;
  start_condition_ = other.start_condition_;
  end_condition_ = other.end_condition_;

}

// p(t) = sum coef_[i] * t^i
double QuinticPolynomial::Evaluate(size_t order, double p) const {
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
    default:
      return 0.0;
  }
}

double QuinticPolynomial::ParamLength() const {
  return param_;
}
size_t QuinticPolynomial::Order() const {
  return order_;
}
double QuinticPolynomial::Coef(size_t order) const {
  ROS_ASSERT(order <= order_);
  return coef_[order];
}

void QuinticPolynomial::SetUpPolynomial(double x0,
                                        double dx0,
                                        double ddx0,
                                        double x1,
                                        double dx1,
                                        double ddx1,
                                        double param) {
  ROS_ASSERT(param > 0.0);
  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = ddx0 / 2.0;

  const double p2 = param * param;
  const double p3 = param * p2;
  const double c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * param - x0) / p3;
  const double c1 = (dx1 - ddx0 * param - dx0) / p2;
  const double c2 = (ddx1 - ddx0) / param;
  coef_[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
  coef_[4] = (-15.0 * c0 + 7.0 * c1 - c2) / param;
  coef_[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
}

}