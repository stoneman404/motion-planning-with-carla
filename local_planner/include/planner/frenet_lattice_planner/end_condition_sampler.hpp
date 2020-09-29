#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_END_CONDITION_SAMPLER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_END_CONDITION_SAMPLER_HPP_
#include <array>
#include <vector>
#include "st_data.hpp"
#include "planner/frenet_lattice_planner/st_graph.hpp"
#include "planning_context.hpp"

namespace planning {

class EndConditionSampler {
 public:

  EndConditionSampler() = default;
  ~EndConditionSampler() = default;
  EndConditionSampler(const std::array<double, 3> &init_s,
                      const std::array<double, 3> &init_d,
                      std::shared_ptr<STGraph> ptr_st_graph);
  /**
   *
   * @param ref_stop_point
   * @return
   */
  std::vector<std::pair<std::array<double, 3>,
                        double>> SampleLonEndConditionForStopping(const double ref_stop_point) const;

  /**
   *
   * @param ref_target_vel
   * @return
   */
  std::vector<std::pair<std::array<double, 3>,
                        double>> SampleLonEndConditionForCruising(const double ref_target_vel) const;

  /**
   *
   * @return
   */
  std::vector<std::pair<std::array<double, 3>, double>> SampleLonEndConditionWithObstacles() const;

  /**
   *
   * @return
   */
  static std::vector<std::pair<std::array<double, 3>, double>> SampleLatEndCondition();

 private:

  /**
   *
   * @param obstacle_id
   * @return
   */
  std::vector<std::pair<STPoint, double>> OvertakeSamplePoints(int obstacle_id) const;

  /**
   *
   * @param obstacle_id
   * @return
   */
  std::vector<std::pair<STPoint, double>> FollowingSamplePoints(int obstacle_id) const;

  double
  /**
   *
   * @param t
   * @return
   */
  double VUpper(double t) const;

  /**
   *
   * @param t
   * @return
   */
  double VLower(double t) const;

  /**
   *
   * @param t
   * @return
   */
  double SUpper(double t) const;

  /**
   *
   * @param t
   * @return
   */
  double SLower(double t) const;

 private:
  std::array<double, 3> init_s_{};
  std::array<double, 3> init_d_{};
  std::shared_ptr<STGraph> ptr_st_graph_;
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_END_CONDITION_SAMPLER_HPP_