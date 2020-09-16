#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_POLYNOMIAL_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_POLYNOMIAL_HPP_
#include <vector>
namespace planning{
class Polynomial{
 public:
  Polynomial() = default;
  virtual ~Polynomial() = default;
  virtual double Evaluate(double t, double order) = 0;

 private:
  size_t order_;
  std::vector<double> params_;



};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_POLYNOMIAL_HPP_
