#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_SPLINE2D_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_SPLINE2D_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>
#include <vector>
namespace planning {

class Spline2d {
 public:
  Spline2d() = default;
  Spline2d(const std::vector<double> &xs, const std::vector<double> &ys, size_t order);
  Spline2d(const std::vector<double> &xs, const std::vector<double> &ys);
  ~Spline2d() = default;
  double ArcLength() const { return this->arc_length_; }
  const std::vector<double> &ChordLength() const { return chord_lengths_; }
  size_t Order() const { return this->order_; }

  /**
   * @brief:
   * @param s
   * @param order
   * @param x
   * @param y
   * @return
   */
  bool Evaluate(double s, double *const x, double *const y) const;

  /**
   * @brief: evaluate first derivative
   * @param s : the arc length at which calc derivative
   * @param dx: x'(s)
   * @param dy: y'(s)
   * @return
   */
  bool EvaluateFirstDerivative(double s, double *const dx, double *const dy) const;
  bool EvaluateSecondDerivative(double s, double *const ddx, double *const ddy) const;
  bool EvaluateThirdDerivative(double s, double *const dddx, double *const dddy) const;

  /**
   * @brief: get the nearest point on spline curve
   * @param x
   * @param y
   * @param nearest_x
   * @param nearest_y
   * @return
   */
  bool GetNearestPointOnSpline(double x, double y,
                               double *const nearest_x,
                               double *const nearest_y,
                               double *const nearest_s) const;

 private:
  /**
   * @brief: calc arc length
   */
  void CalcArcLength();

  /**
   * @brief: calc the arc length at chord length t [0, 1]
   * @param t
   */
  double CalcArcLengthAtT(double t) const;

  double SimpsonIntegral(double t0, double t1) const;

  /**
   * @brief: adaptive simpson integral
   * @param l
   * @param r
   * @param eps
   * @return
   */
  double AdaptiveSimpsonIntegral(double l, double r, double eps) const;

  /**
   * calculate the chord lengths,
   */
  inline void CalcChordLengths();

  /**
   * @brief: given arc length, calculate chord length
   * @param[in] s:  the input arc length
   * @param[out] t: the normalized chord length, which in [0, 1]
   * @return
   */
  bool ArcLengthMapToChordLength(double s, double *const t) const;

  /**
   * @brief: we setup the object function to calculate the nearest point on spline as :
   * D(t) = (x(t) - x)^2 + (y(t) - y)^2
   * @param t: the chord length
   * @param order: the order, order = 1, 2
   * @param x
   * @param y
   * @return
   */
  double CalcDerivativeOfObjectFunction(double t, int order, double x, double y) const;

  /**
   * @brief:
   * @param s
   * @param s1
   * @param s2
   * @param s3
   * @param d1
   * @param d2
   * @param d3
   * @return
   */
  double CalcQ(double s, double s1, double s2, double s3,
               double d1, double d2, double d3) const;

  /**
   * @brief:
   * @param x1
   * @param y1
   * @param x2
   * @param y2
   * @return
   */
  double PointToPointSquaredDistance(double x1, double y1,
                                     double x2, double y2) const;

  /**
   *
   * @param t
   * @param lb
   * @param ub
   * @return
   */
  double Clamp(double t, double lb, double ub) const;

  /**
   * @brief: calc the nearest index
   * @param x
   * @param y
   * @return
   */
  int CalcNearestIndex(double x, double y) const;

 private:
  std::vector<double> xs_;
  std::vector<double> ys_;
  size_t order_ = 5;
  Eigen::Spline2d spline2d_;
  double arc_length_ = 0.0;
  std::vector<double> chord_lengths_;
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_COMMON_INCLUDE_SPLINE2D_HPP_
