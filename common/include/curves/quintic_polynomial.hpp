#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_COMMON_QUINTIC_POLYNOMIAL_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_COMMON_QUINTIC_POLYNOMIAL_HPP_
#include "curves/polynomial.hpp"
#include <array>
namespace common {
class QuinticPolynomial : public Polynomial {
 public:
  QuinticPolynomial() = default;

  QuinticPolynomial(const std::array<double, 3> &start,
                    const std::array<double, 3> &end,
                    double param);

  QuinticPolynomial(double x0, double dx0, double ddx0,
                    double x1, double dx1, double ddx1,
                    double param);

  QuinticPolynomial(const QuinticPolynomial &other);

  void SetParam(double x0, double dx0, double ddx0,
                double x1, double dx1, double ddx1,
                double param);

  ~QuinticPolynomial() override = default;

  double Evaluate(size_t order, double p) const override;

  double ParamLength() const override { return param_; }

  double Coef(size_t order) const override;

  size_t Order() const override { return 5; }

 protected:
  void ComputeCoefficients(double x0, double dx0, double ddx0,
                           double x1, double dx1, double ddx1,
                           double param);

  // f = sum(coef_[i] * x^i), i from 0 to 5
  std::array<double, 6> coef_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_{{0.0, 0.0, 0.0}};
  std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};
};

}
#endif //
