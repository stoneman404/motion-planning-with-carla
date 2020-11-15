#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_COMMON_QUARTIC_POLYNOMIAL_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_COMMON_INCLUDE_COMMON_QUARTIC_POLYNOMIAL_HPP_
#include <array>
#include "curves/polynomial.hpp"
namespace common{
class QuarticPolynomial : public Polynomial {
 public:
  QuarticPolynomial() = default;

  QuarticPolynomial(const std::array<double, 3> &start,
                    const std::array<double, 2> &end,
                    double param);

  QuarticPolynomial(double x0, double dx0, double ddx0,
                    double dx1, double ddx1,
                    double param);

  QuarticPolynomial(const QuarticPolynomial &other);

  ~QuarticPolynomial() override = default;

  /**
   * @brief
   * @param order
   * @param p
   * @return
   */
  double Evaluate(size_t order, double p) const override;

  /**
   * @brief: the param length
   * @return
   */
  double ParamLength() const override;

  /**
   * @brief: the order of this polynomial
   * @return
   */
  size_t Order() const override;

  /**
   * @brief: the coef at order of this polynomial
   * @param order
   * @return
   */
  double Coef(size_t order) const override;

 protected:

  /**
   * @brief: set up the polynomial, that is, calculate the the coefs of polynomial
   * @param x0
   * @param dx0
   * @param ddx0
   * @param dx1
   * @param ddx1
   * @param param
   */
  void SetUpPolynomial(double x0, double dx0, double ddx0,
                       double dx1, double ddx1, double param);
 private:
  std::array<double, 3> start_condition_{0.0, 0.0, 0.0};
  std::array<double, 2> end_condition_{0.0, 0.0};
  // p(t) = sum coef_[i] * t^i
  std::array<double, 5> coef_{0.0, 0.0, 0.0, 0.0, 0.0};
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_QUARTIC_POLYNOMIAL_HPP_
