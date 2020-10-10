#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_QUINTIC_POLYNOMIAL_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_QUINTIC_POLYNOMIAL_HPP_
#include "polynomial.hpp"
namespace planning {
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
  /**
   *
   * @param order
   * @param p
   * @return
   */
  double Evaluate(size_t order, double p) const override;

  /**
   *
   * @return
   */
  double ParamLength() const override;

  /**
   *
   * @return
   */
  size_t Order() const override;

  /**
   *
   * @param order
   * @return
   */
  double Coef(size_t order) const override;

  ~QuinticPolynomial() override = default;

 protected:

  /**
   *
   * @param x0
   * @param dx0
   * @param ddx0
   * @param x1
   * @param dx1
   * @param ddx1
   * @param param
   */
  void SetUpPolynomial(double x0, double dx0, double ddx0,
                       double x1, double dx1, double ddx1,
                       double param);
 private:

  std::array<double, 6> coef_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> start_condition_{0.0, 0.0, 0.0};
  std::array<double, 3> end_condition_{0.0, .0, 0.0};
};

}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_QUINTIC_POLYNOMIAL_HPP_
