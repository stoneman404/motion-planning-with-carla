#include <cassert>
#include "curves/quartic_polynomial.hpp"

namespace common {

QuarticPolynomial::QuarticPolynomial(const std::array<double, 3> &start,
                                     const std::array<double, 2> &end,
                                     double param) : start_condition_(start),
                                                     end_condition_(end) {
  param_ = param;
  order_ = 4;
  SetUpPolynomial(start[0], start[1], start[2], end[0], end[1], param);

}

QuarticPolynomial::QuarticPolynomial(double x0, double dx0, double ddx0, double dx1, double ddx1, double param) {
  param_ = param;
  order_ = 4;
  start_condition_ = {x0, dx0, ddx0};
  end_condition_ = {dx1, ddx1};
  SetUpPolynomial(x0, dx0, ddx0, dx1, ddx1, param);
}

QuarticPolynomial::QuarticPolynomial(const QuarticPolynomial &other) {
  param_ = other.param_;
  order_ = other.order_;
  start_condition_ = other.start_condition_;
  end_condition_ = other.end_condition_;
  coef_ = other.coef_;
}
double QuarticPolynomial::Evaluate(size_t order, double p) const {
  switch (order) {
    case 0: {
      return (((coef_[4] * p + coef_[3]) * p + coef_[2]) * p + coef_[1]) * p +
          coef_[0];
    }
    case 1: {
      return ((4.0 * coef_[4] * p + 3.0 * coef_[3]) * p + 2.0 * coef_[2]) * p +
          coef_[1];
    }
    case 2: {
      return (12.0 * coef_[4] * p + 6.0 * coef_[3]) * p + 2.0 * coef_[2];
    }
    case 3: {
      return 24.0 * coef_[4] * p + 6.0 * coef_[3];
    }
    case 4: {
      return 24.0 * coef_[4];
    }
    default:
      return 0.0;
  }
}

double QuarticPolynomial::ParamLength() const {
  return param_;
}

size_t QuarticPolynomial::Order() const {
  return order_;
}

double QuarticPolynomial::Coef(size_t order) const {
  assert(order <= order_);
  return coef_[order];
}

void QuarticPolynomial::SetUpPolynomial(double x0, double dx0, double ddx0, double dx1, double ddx1, double param) {
  assert(param > 0.0);
  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5 * ddx0;

  double b0 = dx1 - ddx0 * param - dx0;
  double b1 = ddx1 - ddx0;

  double p2 = param * param;
  double p3 = p2 * param;

  coef_[3] = (3 * b0 - b1 * param) / (3 * p2);
  coef_[4] = (-2 * b0 + b1 * param) / (4 * p3);
}

}