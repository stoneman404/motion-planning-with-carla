#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_POLYNOMIAL_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_POLYNOMIAL_HPP_
#include <vector>
#include <cstdint>
namespace planning{
class Polynomial {
 public:
  Polynomial() = default;
  virtual ~Polynomial() = default;
  virtual double Evaluate(size_t order, double param) const = 0;
  virtual double ParamLength() const = 0;
  virtual size_t Order() const = 0;
  virtual double Coef(size_t order) const = 0;

 protected:
  double param_ = 0.0;
  size_t order_ = 0;
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_POLYNOMIAL_HPP_
