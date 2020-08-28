#include "spline2d.hpp"
#include <ros/ros.h>
namespace planning {

Spline2d::Spline2d(const std::vector<double> &xs,
                   const std::vector<double> &ys)
    : xs_(xs),
      ys_(ys) {
  assert(xs.size() == ys.size());
  assert(xs.size() > order_);
  Eigen::MatrixXd points(2, static_cast<Eigen::Index>(xs.size()));
  for (size_t i = 0; i < xs.size(); ++i) {
    points(0, i) = xs[i];
    points(1, i) = ys[i];
  }
  spline2d_ = Eigen::SplineFitting<Eigen::Spline2d>::Interpolate(
      points, static_cast<Eigen::Index >(order_));
  CalcChordLengths();
  CalcArcLength();
}

Spline2d::Spline2d(const std::vector<double> &xs,
                   const std::vector<double> &ys, size_t order)
    : xs_(xs),
      ys_(ys),
      order_(order) {
  assert(xs.size() == ys.size());
  assert(xs.size() > order_);
  Eigen::MatrixXd points(2, static_cast<Eigen::Index>(xs.size()));
  for (size_t i = 0; i < xs.size(); ++i) {
    points(0, i) = xs[i];
    points(1, i) = ys[i];
  }
  spline2d_ = Eigen::SplineFitting<Eigen::Spline2d>::Interpolate(
      points, static_cast<Eigen::Index >(order_));
  CalcChordLengths();
  CalcArcLength();
}

bool Spline2d::Evaluate(double s,
                        double *const x,
                        double *const y) const {

  if (s < 1e-3) {
    *x = spline2d_(0.0)(0, 0);
    *y = spline2d_(0.0)(1, 0);
    return true;
  } else if (s > arc_length_ - 1e-3) {
    *x = spline2d_(1.0)(0, 0);
    *y = spline2d_(1.0)(1, 0);
    return true;
  } else {
    double t = 0;
    ArcLengthMapToChordLength(s, &t);
    *x = spline2d_(t)(0, 0);
    *y = spline2d_(t)(1, 0);
    return true;
  }
}

bool Spline2d::EvaluateFirstDerivative(double s,
                                       double *const dx,
                                       double *const dy) const {
  if (s < 1e-3) {
    *dx = spline2d_.derivatives(0.0, 1)(0, 1);
    *dy = spline2d_.derivatives(0.0, 1)(1, 1);
    return true;
  } else if (s > arc_length_ - 1e-3) {
    *dx = spline2d_.derivatives(1.0, 1)(0, 1);
    *dy = spline2d_.derivatives(1.0, 1)(1, 1);
    return true;
  } else {
    double t = 0;
    ArcLengthMapToChordLength(s, &t);
    *dx = spline2d_.derivatives(t, 1)(0, 1);
    *dy = spline2d_.derivatives(t, 1)(1, 1);
    return true;
  }
}

bool Spline2d::EvaluateSecondDerivative(double s,
                                        double *const ddx,
                                        double *const ddy) const {

  if (s < 1e-3) {
    *ddx = spline2d_.derivatives(0.0, 2)(0, 2);
    *ddy = spline2d_.derivatives(0.0, 2)(1, 2);
    return true;
  } else if (s > arc_length_ - 1e-3) {
    *ddx = spline2d_.derivatives(1.0, 2)(0, 2);
    *ddy = spline2d_.derivatives(1.0, 2)(1, 2);
    return true;
  } else {
    double t = 0;
    ArcLengthMapToChordLength(s, &t);
    *ddx = spline2d_.derivatives(t, 2)(0, 2);
    *ddy = spline2d_.derivatives(t, 2)(1, 2);
    return true;
  }
}

bool Spline2d::EvaluateThirdDerivative(double s,
                                       double *const dddx,
                                       double *const dddy) const {
  if (s < 1e-3) {
    *dddx = spline2d_.derivatives(0.0, 3)(0, 3);
    *dddy = spline2d_.derivatives(0.0, 3)(1, 3);
    return true;
  } else if (s > arc_length_ - 1e-3) {
    *dddx = spline2d_.derivatives(1.0, 3)(0, 3);
    *dddy = spline2d_.derivatives(1.0, 3)(1, 3);
    return true;
  } else {
    double t = 0;
    ArcLengthMapToChordLength(s, &t);
    *dddx = spline2d_.derivatives(t, 3)(0, 3);
    *dddy = spline2d_.derivatives(t, 3)(1, 3);
    return true;
  }
}

void Spline2d::CalcArcLength() {
  const double eps = 0.001;
  arc_length_ = AdaptiveSimpsonIntegral(0, 1, eps);
}

void Spline2d::CalcChordLengths() {
  const size_t np = xs_.size();
  chord_lengths_.clear();
  chord_lengths_.reserve(xs_.size());
  double chord_length = 0.0;
  chord_lengths_.push_back(chord_length);
  double prev_x = xs_[0];
  double prev_y = ys_[0];
  for (int i = 1; i < np; i++) {
    const double dx = xs_[i] - prev_x;
    const double dy = ys_[i] - prev_y;
    const double ds = std::hypot(dx, dy);
    chord_length += ds;
    chord_lengths_.push_back(chord_length);
    prev_x = xs_[i];
    prev_y = ys_[i];
  }
}

double Spline2d::CalcArcLengthAtT(double t) const {
  if (t < 1e-3) {
    return 0.0;
  }
  if (t > arc_length_ - 1e-3) {
    return arc_length_;
  }
  const double eps = 0.001;
  return AdaptiveSimpsonIntegral(0.0, t, eps);
}

bool Spline2d::ArcLengthMapToChordLength(double s, double *const t) const {
  int max_iter = 10;
  int iter = 0;
  const double iter_eps = 0.001;
  double approx_t = s / arc_length_;
  double pre_approx_t;
  while (iter < max_iter) {
    double approx_s = CalcArcLengthAtT(approx_t);
    double d = approx_s - s;
    if (std::fabs(d) < iter_eps) {
      break;
    }
    double first_derivative = std::hypot(spline2d_.derivatives(approx_t, 1)(0, 1),
                                         spline2d_.derivatives(approx_t, 1)(1, 1));
    double second_derivative = std::hypot(spline2d_.derivatives(approx_t, 2)(0, 2),
                                          spline2d_.derivatives(approx_t, 2)(1, 2));
    double numerator = d * first_derivative;
    double denominator = d * second_derivative + first_derivative * first_derivative;
    pre_approx_t = approx_t;
    approx_t -= numerator / denominator;
    if (std::fabs(pre_approx_t - approx_t) < iter_eps) {
      break;
    }
    iter++;
  }
  *t = approx_t;
  return true;
}

double Spline2d::SimpsonIntegral(double l, double r) const {
  assert(r > l);
  const double mid_l = (2.0 * l + r) / 3.0;
  const double mid_r = (l + 2.0 * r) / 3.0;
  const double d_l = std::hypot(spline2d_.derivatives(l, 1)(0, 1),
                                spline2d_.derivatives(l, 1)(1, 1));
  const double d_mid_l = std::hypot(spline2d_.derivatives(mid_l, 1)(0, 1),
                                    spline2d_.derivatives(mid_l, 1)(1, 1));
  const double d_mid_r = std::hypot(spline2d_.derivatives(mid_r, 1)(0, 1),
                                    spline2d_.derivatives(mid_r, 1)(1, 1));
  const double d_r = std::hypot(spline2d_.derivatives(r, 1)(0, 1),
                                spline2d_.derivatives(r, 1)(1, 1));
  return (r - l) / 8.0 * (d_l + 3.0 * d_mid_l + 3.0 * d_mid_r + d_r);
}

double Spline2d::AdaptiveSimpsonIntegral(double l, double r, double eps) const {

  double mid = (l + r) / 2.0;

  double st = SimpsonIntegral(l, r);
  double sl = SimpsonIntegral(l, mid);
  double sr = SimpsonIntegral(mid, r);
  if (std::fabs(sl + sr - st) < 15.0 * eps) {
    return sl + sr + (sl + sr - st) / 15.0;
  } else {
    return AdaptiveSimpsonIntegral(l, mid, eps / 2.0) +
        AdaptiveSimpsonIntegral(mid, r, eps / 2.0);
  }
}


bool Spline2d::GetNearestPointOnSpline(double x, double y,
                                       double *const nearest_x,
                                       double *const nearest_y,
                                       double *const nearest_s) const {
  // 1. prepared, set the init s1, s2, s3
  // note: here we use the chord length rather than arc length for eliminating the calculate time;

  // t1, t2, t3, tk_star  refer to: Robust and Efficient Computation of the
  // Closest Point on a Spline Curve
  double t_opt;
  int min_index = CalcNearestIndex(x, y);
  double t1 = chord_lengths_[min_index] / chord_lengths_.back();
  Clamp(t1, 0, 1);
//  int t2_index = min_index - 1 < 0 ? 0 : min_index - 1;
//  int t3_index = min_index + 1 > chord_lengths_.size() - 1 ? chord_lengths_.size() - 1 : min_index + 1;
//  double t2 = chord_lengths_[t2_index] / chord_lengths_.back();
//  double t3 = chord_lengths_[t3_index] / chord_lengths_.back();
  double t2, t3;
  if (t1 < 0.5){
    t2 = std::min(t1 + 0.1, 1.0);
    t3 = std::min(t1 + 0.2, 1.0);
  }else{
    t2 = std::max(t1 - 0.1, 0.0);
    t3 = std::max(t1 - 0.2, 0.0);
  }

  std::array<double, 3> ts{t1, t2, t3};
//  std::array<double, 3> ts{0.1, 0.5,0.9};
  std::array<double, 4> ps{0.0, 0.0, 0.0, 0.0};
  double term_cond = 1e-6;
  const int max_iter = 10; // max iter time
  int iter = 0;
  double t_last = 1;
  while (iter < max_iter) {
    // 1. QP step
    const double t23 = ts[1] - ts[2];
    const double t31 = ts[2] - ts[0];
    const double t12 = ts[0] - ts[1];
    const double y23 = ts[1] * ts[1] - ts[2] * ts[2];
    const double y31 = ts[2] * ts[2] - ts[0] * ts[0];
    const double y12 = ts[0] * ts[0] - ts[1] * ts[1];

    auto xy1 = spline2d_(ts[0]);
    auto xy2 = spline2d_(ts[1]);
    auto xy3 = spline2d_(ts[2]);
    double d1 = SquaredPointToPointDistance(xy1(0, 0), xy1(1, 0), x, y);
    double d2 = SquaredPointToPointDistance(xy2(0, 0), xy2(1, 0), x, y);
    double d3 = SquaredPointToPointDistance(xy3(0, 0), xy3(1, 0), x, y);

    t_opt = 0.5 * (y23 * d1 + y31 * d2 + y12 * d3) /
        (t23 * d1 + t31 * d2 + t12 * d3);
//    std::cout << "t_opt : " << t_opt << std::endl;
    if (isnan(t_opt)) {
      t_opt = t_last;
      break;
    }
    ps[0] = CalcQ(ts[0], ts[0], ts[1], ts[2], d1, d2, d3);
    ps[1] = CalcQ(ts[1], ts[0], ts[1], ts[2], d1, d2, d3);
    ps[2] = CalcQ(ts[2], ts[0], ts[1], ts[2], d1, d2, d3);
    ps[3] = CalcQ(t_opt, ts[0], ts[1], ts[2], d1, d2, d3);


    // 2. Newton method step
    const double D_prime_t = CalcDerivativeOfObjectFunction(t_opt, 1, x, y);
    const double D_prime_prime_t = CalcDerivativeOfObjectFunction(t_opt, 2, x, y);

    t_opt -=  D_prime_t / D_prime_prime_t;
    if (isnan(t_opt)) {
//     std::cout << "in newton step the opt is nan, D'(t) = " << D_prime_t
//     <<  "D''(t)" << D_prime_prime_t << std::endl;
//      return false;
      t_opt = t_last;
      break;
    }
    t_opt = Clamp(t_opt, 0, 1);
    if (std::fabs(t_opt - t_last) / t_last < term_cond) {
      // satisfy the terminate condition, break;
      break;
    }

    t_last = t_opt;
    // 3. update the s1, s2, s3 by replace the value of which has max ps value.
    int biggest = 0;
    for (int i = 0; i < ps.size() - 1; ++i) {
      if (ps[i] > ps[biggest]) {
        biggest = i;
      }
    }

//    std::cout << "biggest : " << biggest << std::endl;
    ts[biggest] = t_opt;
//    std::cout << "ts: " << ts[0] << ", " << ts[1] << ", " << ts[2] << " t_opt: " << t_opt << std::endl;

    iter++;
//    std::cout << "iter: " << iter << std::endl;
  }

  if (iter >= max_iter && std::fabs(ts[3] - t_last) > term_cond) {
    return false;
  }

  *nearest_x = spline2d_(t_opt)(0, 0);
  *nearest_y = spline2d_(t_opt)(1, 0);
  *nearest_s = CalcArcLengthAtT(t_opt);
  return true;
}

double Spline2d::SquaredPointToPointDistance(double x1, double y1,
                                             double x2, double y2) const {
  return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

double Spline2d::CalcQ(double s, double s1, double s2, double s3,
                       double d1, double d2, double d3) const {
  const double p1 = (s - s2) * (s - s3) / ((s1 - s2) * (s1 - s3)) * d1;
  const double p2 = (s - s1) * (s - s3) / ((s2 - s1) * (s2 - s3)) * d2;
  const double p3 = (s - s1) * (s - s2) / ((s3 - s1) * (s3 - s2)) * d3;
  return p1 + p2 + p3;
}

double Spline2d::CalcDerivativeOfObjectFunction(double t, int order, double x, double y) const {
  const auto xy_t = spline2d_(t);
  const auto xy_dt = spline2d_.derivatives(t, 1);
  const auto xy_ddt = spline2d_.derivatives(t, 2);
  const double xt = xy_t(0, 0);
  const double yt = xy_t(1, 0);
  const double dxdt = xy_dt(0, 1);
  const double dydt = xy_dt(1, 1);
  const double dxddt = xy_ddt(0, 2);
  const double dyddt = xy_ddt(1, 2);
  double result;
  switch (order) {
    case 1: {
      result = 2.0 * ((xt - x) * dxdt + (yt - y) * dydt);
      break;
    }
    case 2: {
      result = 2.0 * (dxdt * dxdt + dxddt * (xt - x) + dydt * dydt + dyddt * (yt - y));
      break;
    }
    default:break;
  }
  return result;

}

double Spline2d::Clamp(double t, double lb, double ub) const {
  if (t < lb) {
    return lb;
  } else if (t > ub) {
    return ub;
  } else {
    return t;
  }
}

int Spline2d::CalcNearestIndex(double x, double y) const {
  double min_dist = std::numeric_limits<double>::max();
  int min_index = 0;
  for (size_t i = 0; i < xs_.size(); ++i){
    const double sqrt_dist = SquaredPointToPointDistance(x, y, xs_[i], ys_[i]);
    if (sqrt_dist < min_dist){
      min_dist = sqrt_dist;
      min_index = i;
    }
  }
  return min_index;
}
}