#include "pure_pursuit_control.hpp"

namespace planning{

bool PurePursuitControl::CalculateDesiredSteer(const double wheel_base_len,
                                               const double angle_diff,
                                               const double lookahead_dist,
                                               double *steer) {
  *steer = std::atan2(2.0 * wheel_base_len * std::sin(angle_diff), lookahead_dist);
  return true;
}
}