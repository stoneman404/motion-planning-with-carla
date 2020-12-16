#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MOTION_PLANNER_FRENENT_LATTICE_PLANNER_END_CONDITION_SAMPLER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MOTION_PLANNER_FRENENT_LATTICE_PLANNER_END_CONDITION_SAMPLER_HPP_
#include <array>
#include <vector>
#include "math/frenet_frame.hpp"
#include "obstacle_manager/st_graph.hpp"
#include "reference_line/reference_line.hpp"
namespace planning {

class EndConditionSampler {
 public:

  EndConditionSampler() = default;

  ~EndConditionSampler() = default;

  EndConditionSampler(const std::array<double, 3> &init_s,
                      const std::array<double, 3> &init_d,
                      std::shared_ptr<ReferenceLine> ptr_ref_line,
                      const std::vector<std::shared_ptr<Obstacle>> &ptr_obstacles,
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
  std::vector<std::pair<std::array<double, 3>, double>> SampleLonEndConditionWithSTGraph() const;

  /**
   *
   * @return
   */
  std::vector<std::pair<std::array<double, 3>, double>> SampleLatEndCondition() const;

 private:

  /**
   *
   * @param obstacle_id
   * @return
   */
  std::vector<std::pair<common::STPoint, double>> OvertakeSamplePoints(int obstacle_id) const;

  /**
   *
   * @param obstacle_id
   * @return
   */
  std::vector<std::pair<common::STPoint, double>> FollowingSamplePoints(int obstacle_id) const;

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

  /**
   * @brief: project the obstacle's velocity at time t to reference line,
   * @param obstacle_id
   * @param s
   * @param t
   * @param ptr_ref_line
   * @return
   */
  double GetObstacleSpeedAlongReferenceLine(int obstacle_id, double s, double t,
                                            const std::shared_ptr<ReferenceLine> &ptr_ref_line) const;

 private:
  std::array<double, 3> init_s_{};
  std::array<double, 3> init_d_{};
  std::shared_ptr<ReferenceLine> ptr_ref_line_;
  std::unordered_map<int, std::shared_ptr<Obstacle>> obstacles_;
  std::shared_ptr<STGraph> ptr_st_graph_;
};
}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_END_CONDITION_SAMPLER_HPP_