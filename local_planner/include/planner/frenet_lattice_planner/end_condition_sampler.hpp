#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_END_CONDITION_SAMPLER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_END_CONDITION_SAMPLER_HPP_
#include <array>
#include <vector>
#include "st_data.hpp"
#include "planning_context.hpp"

namespace planning {

class EndConditionSampler {
 public:
  using State = std::array<double, 3>;
  using EndCondition = std::pair<State, double>;

  EndConditionSampler() = default;
  ~EndConditionSampler() = default;
  EndConditionSampler(const std::array<double, 3> &init_s, const std::array<double, 3> &init_d);
  /**
   *
   * @param ref_stop_point
   * @return
   */
  std::vector<EndCondition> SampleLonEndConditionForStopping(const double ref_stop_point) const;

  /**
   *
   * @param ref_target_vel
   * @return
   */
  std::vector<EndCondition> SampleLonEndConditionForCruising(const double ref_target_vel) const;

  /**
   *
   * @return
   */
  std::vector<EndCondition> SampleLonEndConditionForSTPoint() const;

  /**
   *
   * @return
   */
  std::vector<EndCondition> SampleLatEndCondition() const;
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_END_CONDITION_SAMPLER_HPP_