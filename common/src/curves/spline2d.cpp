#include "curves/spline2d.hpp"
#include <ros/ros.h>
namespace common {

Spline2d::Spline2d(const std::vector<double> &xs,
                   const std::vector<double> &ys) : xs_(xs), ys_(ys) {
  assert(xs.size() == ys.size());
  assert(xs_.size() > order_);
  this->CalcChordLengths();
  assert(chord_lengths_.size() == xs.size());
  x_spline_ = spline();
  y_spline_ = spline();
  x_spline_.set_points(chord_lengths_, xs_);
  y_spline_.set_points(chord_lengths_, ys_);
  CalcArcLength();
}

Spline2d::Spline2d(const std::vector<double> &xs,
                   const std::vector<double> &ys, size_t order)
    : xs_(xs), ys_(ys), order_(order) {
  assert(xs.size() == ys.size());
  assert(xs_.size() > order_);
  this->CalcChordLengths();
  assert(chord_lengths_.size() == xs.size());
  x_spline_ = spline();
  y_spline_ = spline();
  x_spline_.set_points(chord_lengths_, xs_);
  y_spline_.set_points(chord_lengths_, ys_);
  CalcArcLength();
}

bool Spline2d::Evaluate(double s,
                        double *const x,
                        double *const y) const {

  if (s < 1e-3) {
    *x = x_spline_(0.0);
    *y = y_spline_(0.0);
    return true;
  } else if (s > arc_length_ - 1e-3) {
    *x = x_spline_(arc_length_);
    *y = y_spline_(arc_length_);
    return true;
  } else {
    *x = x_spline_(s);
    *y = y_spline_(s);
    return true;
  }
}

bool Spline2d::EvaluateFirstDerivative(double s,
                                       double *const dx,
                                       double *const dy) const {
  if (s < 1e-3) {
    *dx = x_spline_.deriv(1, 0.0);
    *dy = y_spline_.deriv(1, 0.0);
    return true;
  } else if (s > arc_length_ - 1e-3) {
    *dx = x_spline_.deriv(1, arc_length_);
    *dy = y_spline_.deriv(1, arc_length_);
    return true;
  } else {
    *dx = x_spline_.deriv(1, arc_length_);
    *dy = y_spline_.deriv(1, arc_length_);
    return true;
  }
}

bool Spline2d::EvaluateSecondDerivative(double s,
                                        double *const ddx,
                                        double *const ddy) const {

  if (s < 1e-3) {
    *ddx = x_spline_.deriv(2, 0.0);
    *ddy = y_spline_.deriv(2, 0.0);
    return true;
  } else if (s > arc_length_ - 1e-3) {
    *ddx = x_spline_.deriv(2, arc_length_);
    *ddy = y_spline_.deriv(2, arc_length_);
    return true;
  } else {
    *ddx = x_spline_.deriv(2, s);
    *ddy = y_spline_.deriv(2, s);
    return true;
  }
}

bool Spline2d::EvaluateThirdDerivative(double s,
                                       double *const dddx,
                                       double *const dddy) const {
  if (s < 1e-3) {
    *dddx = x_spline_.deriv(3, 0.0);
    *dddy = y_spline_.deriv(3, 0.0);
    return true;
  } else if (s > arc_length_ - 1e-3) {
    *dddx = x_spline_.deriv(3, arc_length_);
    *dddy = y_spline_.deriv(3, arc_length_);
    return true;
  } else {
    *dddx = x_spline_.deriv(3, s);
    *dddy = y_spline_.deriv(3, s);
    return true;
  }
}

void Spline2d::CalcArcLength() {
//  const double step = 0.001;
//  double t = step;
//  double last_x = spline2d_(0.0)(0, 0);
//  double last_y = spline2d_(0.0)(1, 0);
//  double arc_length = 0.0;
//  while (t < 1.0 + step) {
//    if (t > 1.0) {
//      t = 1.0;
//    }
//    if (t < 1e-6) {
//      t = 0.0;
//    }
//    auto xy = spline2d_(t);
//    double x = xy(0, 0);
//    double y = xy(1, 0);
//    double delta_s = std::hypot(x - last_x, y - last_y);
//    arc_length += delta_s;
//    last_x = x;
//    last_y = y;
//    t += step;
//
//  }
  arc_length_ = chord_lengths_.back();

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
  return t * arc_length_;
//  return AdaptiveSimpsonIntegral(0.0, t, eps);
}

bool Spline2d::ArcLengthMapToChordLength(double s, double *const t) const {
//  int max_iter = 10;
//  int iter = 0;
//  const double iter_eps = 0.001;
//  double approx_t = s / arc_length_;
//  double pre_approx_t;
//  while (iter < max_iter) {
//    double approx_s = CalcArcLengthAtT(approx_t);
//    double d = approx_s - s;
//    if (std::fabs(d) < iter_eps) {
//      break;
//    }
//    double first_derivative = std::hypot(spline2d_.derivatives(approx_t, 1)(0, 1),
//                                         spline2d_.derivatives(approx_t, 1)(1, 1));
//    double second_derivative = std::hypot(spline2d_.derivatives(approx_t, 2)(0, 2),
//                                          spline2d_.derivatives(approx_t, 2)(1, 2));
//    double numerator = d * first_derivative;
//    double denominator = d * second_derivative + first_derivative * first_derivative;
//    pre_approx_t = approx_t;
//    approx_t -= numerator / denominator;
//    if (std::fabs(pre_approx_t - approx_t) < iter_eps) {
//      break;
//    }
//    iter++;
//  }
//  *t = approx_t;
  *t = s / arc_length_;
  return true;
}

//double Spline2d::SimpsonIntegral(double l, double r) const {
//  assert(r > l);
//  const double mid_l = (2.0 * l + r) / 3.0;
//  const double mid_r = (l + 2.0 * r) / 3.0;
//  const double d_l = std::hypot(spline2d_.derivatives(l, 1)(0, 1),
//                                spline2d_.derivatives(l, 1)(1, 1));
//  const double d_mid_l = std::hypot(spline2d_.derivatives(mid_l, 1)(0, 1),
//                                    spline2d_.derivatives(mid_l, 1)(1, 1));
//  const double d_mid_r = std::hypot(spline2d_.derivatives(mid_r, 1)(0, 1),
//                                    spline2d_.derivatives(mid_r, 1)(1, 1));
//  const double d_r = std::hypot(spline2d_.derivatives(r, 1)(0, 1),
//                                spline2d_.derivatives(r, 1)(1, 1));
//  return (r - l) / 8.0 * (d_l + 3.0 * d_mid_l + 3.0 * d_mid_r + d_r);
//}

//double Spline2d::AdaptiveSimpsonIntegral(double l, double r, double eps) const {
//
//  double mid = (l + r) / 2.0;
//
//  double st = SimpsonIntegral(l, r);
//  double sl = SimpsonIntegral(l, mid);
//  double sr = SimpsonIntegral(mid, r);
//  if (std::fabs(sl + sr - st) < 15.0 * eps) {
//    return sl + sr + (sl + sr - st) / 15.0;
//  } else {
//    return AdaptiveSimpsonIntegral(l, mid, eps / 2.0) +
//        AdaptiveSimpsonIntegral(mid, r, eps / 2.0);
//  }
//}

bool Spline2d::GetNearestPointOnSpline(double x, double y,
                                       double *const nearest_x,
                                       double *const nearest_y,
                                       double *const nearest_s) const {
  // 1. prepared, set the init s1, s2, s3
  // note: here we use the chord length rather than arc length for eliminating the calculate time;

  // t1, t2, t3, tk_star  refer to: Robust and Efficient Computation of the
  // Closest Point on a Spline Curve
  double s_opt;
  int min_index = this->CalcNearestIndex(x, y);
//  double t1 = chord_lengths_[min_index] / chord_lengths_.back();
  double s1 = chord_lengths_[min_index];
  Clamp(s1, chord_lengths_[0], chord_lengths_.back());
//  double t2, t3;
  double s2, s3;
//  if (t1 < 0.5) {
//
//    t2 = std::min(t1 + 0.1, 1.0);
//    t3 = std::min(t1 + 0.2, 1.0);
//  } else {
//    t2 = std::max(t1 - 0.1, 0.0);
//    t3 = std::max(t1 - 0.2, 0.0);
//  }
  if (s1 < 0.5 * chord_lengths_.back()) {
    s2 = std::min(s1 + 1.0, chord_lengths_.back());
    s3 = std::min(s1 + 2.0, chord_lengths_.back());
  } else {
    s2 = std::max(s1 - 1.0, 0.0);
    s3 = std::max(s1 - 2.0, 0.0);
  }

  std::array<double, 3> ss{s1, s2, s3};
  std::array<double, 4> ps{0.0, 0.0, 0.0, 0.0};
  double term_cond = 1e-6;
  const int max_iter = 10; // max iter time
  int iter = 0;
  double s_last = 1;
  while (iter < max_iter) {
    // 1. QP step
    const double s23 = ss[1] - ss[2];
    const double s31 = ss[2] - ss[0];
    const double s12 = ss[0] - ss[1];
    const double y23 = ss[1] * ss[1] - ss[2] * ss[2];
    const double y31 = ss[2] * ss[2] - ss[0] * ss[0];
    const double y12 = ss[0] * ss[0] - ss[1] * ss[1];

//    auto xy1 = spline2d_(ts[0]);
//    auto xy2 = spline2d_(ts[1]);
//    auto xy3 = spline2d_(ts[2]);
    double x1 = x_spline_(ss[0]);
    double y1 = y_spline_(ss[0]);
    double x2 = x_spline_(ss[1]);
    double y2 = y_spline_(ss[1]);
    double x3 = x_spline_(ss[2]);
    double y3 = y_spline_(ss[2]);

    double d1 = PointToPointSquaredDistance(x1, y1, x, y);
    double d2 = PointToPointSquaredDistance(x2, y2, x, y);
    double d3 = PointToPointSquaredDistance(x3, y3, x, y);

    s_opt = 0.5 * (y23 * d1 + y31 * d2 + y12 * d3) /
        (s23 * d1 + s31 * d2 + s12 * d3);
//    std::cout << "t_opt : " << t_opt << std::endl;
    if (isnan(s_opt)) {
      s_opt = s_last;
      break;
    }
    ps[0] = CalcQ(ss[0], ss[0], ss[1], ss[2], d1, d2, d3);
    ps[1] = CalcQ(ss[1], ss[0], ss[1], ss[2], d1, d2, d3);
    ps[2] = CalcQ(ss[2], ss[0], ss[1], ss[2], d1, d2, d3);
    ps[3] = CalcQ(s_opt, ss[0], ss[1], ss[2], d1, d2, d3);


    // 2. Newton method step
    const double D_prime_t = CalcDerivativeOfObjectFunction(s_opt, 1, x, y);
    const double D_prime_prime_t = CalcDerivativeOfObjectFunction(s_opt, 2, x, y);

    s_opt -= D_prime_t / D_prime_prime_t;
    if (isnan(s_opt)) {
//     std::cout << "in newton step the opt is nan, D'(t) = " << D_prime_t
//     <<  "D''(t)" << D_prime_prime_t << std::endl;
//      return false;
      s_opt = s_last;
      break;
    }
    s_opt = Clamp(s_opt, chord_lengths_[0], chord_lengths_.back());
    if (std::fabs(s_opt - s_last) / s_last < term_cond) {
      // satisfy the terminate condition, break;
      break;
    }

    s_last = s_opt;
    // 3. update the s1, s2, s3 by replace the value of which has max ps value.
    int biggest = 0;
    for (int i = 0; i < ps.size() - 1; ++i) {
      if (ps[i] > ps[biggest]) {
        biggest = i;
      }
    }

//    std::cout << "biggest : " << biggest << std::endl;
    ss[biggest] = s_opt;
//    std::cout << "ts: " << ts[0] << ", " << ts[1] << ", " << ts[2] << " t_opt: " << t_opt << std::endl;

    iter++;
//    std::cout << "iter: " << iter << std::endl;
  }

  if (iter >= max_iter && std::fabs(ss[3] - s_last) > term_cond) {
    return false;
  }

  *nearest_x = x_spline_(s_opt);
  *nearest_y = y_spline_(s_opt);
  *nearest_s = s_opt;
  return true;
}

double Spline2d::PointToPointSquaredDistance(double x1, double y1,
                                             double x2, double y2) {
  return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

double Spline2d::CalcQ(double s, double s1, double s2, double s3,
                       double d1, double d2, double d3) {
  const double p1 = (s - s2) * (s - s3) / ((s1 - s2) * (s1 - s3)) * d1;
  const double p2 = (s - s1) * (s - s3) / ((s2 - s1) * (s2 - s3)) * d2;
  const double p3 = (s - s1) * (s - s2) / ((s3 - s1) * (s3 - s2)) * d3;
  return p1 + p2 + p3;
}

double Spline2d::CalcDerivativeOfObjectFunction(double t, int order, double x, double y) const {
//  const auto xy_s = spline2d_(t);
//  const auto xy_ds = spline2d_.derivatives(t, 1);
//  const auto xy_dds = spline2d_.derivatives(t, 2);
  const double xt = x_spline_(t);
  const double yt = y_spline_(t);
  const double dxdt = x_spline_.deriv(1, t);
  const double dydt = y_spline_.deriv(1, t);

  double result = 0;
  switch (order) {
    case 1: {
      result = 2.0 * ((xt - x) * dxdt + (yt - y) * dydt);
      break;
    }
    case 2: {
      const double dxddt = x_spline_.deriv(2, t);
      const double dyddt = y_spline_.deriv(2, t);
      result = 2.0 * (dxdt * dxdt + dxddt * (xt - x) + dydt * dydt + dyddt * (yt - y));
      break;
    }
    default:break;
  }
  return result;

}

double Spline2d::Clamp(double t, double lb, double ub) {
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
  for (size_t i = 0; i < xs_.size(); ++i) {
    const double sqrt_dist = std::hypot(x - xs_[i], y - ys_[i]);
    if (sqrt_dist < min_dist) {
      min_dist = sqrt_dist;
      min_index = i;
    }
  }
  return min_index;
}
}