#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MOTION_PLANNER_FRENET_LATTICE_PLANNER_FRENET_LATTICE_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MOTION_PLANNER_FRENET_LATTICE_PLANNER_FRENET_LATTICE_PLANNER_HPP_

#include <planning_msgs/TrajectoryPoint.h>
#include "motion_planner/trajectory_planner.hpp"
#include "planning_context.hpp"
#include "st_graph.hpp"
#include "end_condition_sampler.hpp"
#include "quartic_polynomial.hpp"
#include "quintic_polynomial.hpp"

namespace planning {

class FrenetLatticePlanner : public TrajectoryPlanner {
 public:

  FrenetLatticePlanner() = default;

  ~FrenetLatticePlanner() override = default;

  /**
   * @brief: main function
   * @param[in] init_trajectory_point
   * @param[in] reference_lines
   * @param[in] maneuver_goal
   * @param[out] pub_trajectory
   * @return
   */
  bool Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
               const ManeuverGoal &maneuver_goal,
               std::shared_ptr<planning_msgs::Trajectory> pub_trajectory) override;

 protected:

  /**
   * @brief: planning on one reference line
   * @param[in] init_trajectory_point
   * @param[in] ptr_ref_line
   * @param[in] maneuver_goal
   * @param[out] pub_trajectory
   * @return
   */
  bool Plan(const planning_msgs::TrajectoryPoint &init_trajectory_point,
            size_t index,
            const ManeuverInfo &maneuver_info,
            std::vector<std::shared_ptr<planning_msgs::Trajectory>> *traj_on_ref_line) const;

  /**
   * @brief: generate lon trajectories and lat trajectories
   * @param maneuver_goal: maneuver goal, comes from maneuver planner
   * @param[out] ptr_lon_traj_vec: lon trajectories
   * @param[out] ptr_lat_traj_vec: lat trajectories
   */
  void GenerateTrajectories(const ManeuverGoal &maneuver_goal,
                            std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec,
                            std::vector<std::shared_ptr<Polynomial>> *ptr_lat_traj_vec) const;

  /**
   * @brief: combine the lon and lat trajectories
   * @param ptr_ref_line: reference line
   * @param lon_traj_vec: lon trajectories
   * @param lat_traj_vec: lat trajectories
   * @param ptr_combined_pub_traj: combined trajectory
   * @return
   */
  static bool CombineTrajectories(std::shared_ptr<ReferenceLine> ptr_ref_line,
                                  const Polynomial &lon_traj_vec, const Polynomial &lat_traj_vec,
                                  std::shared_ptr<planning_msgs::Trajectory> ptr_combined_pub_traj);

 private:

  /**
   * @brief: generate lat polynomial trajectories
   * @param ptr_lat_traj_vec
   */
  void GenerateLatTrajectories(std::vector<std::shared_ptr<Polynomial>> *ptr_lat_traj_vec) const;

  /**
   *
   * @param maneuver_goal
   * @param ptr_lon_traj_vec
   */
  void GenerateLonTrajectories(const ManeuverGoal &maneuver_goal,
                               std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const;

  /**
   * @brief: generate cruising lon trajectories
   * @param cruise_speed:
   * @param ptr_lon_traj_vec
   */
  void GenerateCruisingLonTrajectories(double cruise_speed,
                                       std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const;

  /**
   * @brief: generate stopping lon trajectories
   * @param stop_s: stop position
   * @param ptr_lon_traj_vec
   */
  void GenerateStoppingLonTrajectories(double stop_s,
                                       std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const;

  /**
   * @brief: generate overtake and following lon trajectories
   * @param ptr_lon_traj_vec
   */
  void GenerateOvertakeAndFollowingLonTrajectories(std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const;

  /**
   * @brief: generate polynomial trajectories, quartic_polynomial or quintic_polynomial
   * @param[in] init_condition: initial conditions
   * @param[in] end_conditions: end conditions
   * @param[in] order: order: 4-->quartic polynomial, 5-->quintic polynomial
   * @param[out] ptr_traj_vec: polynomial trajectories
   */
  static void GeneratePolynomialTrajectories(const std::array<double, 3> &init_condition,
                                             const std::vector<std::pair<std::array<double, 3>,
                                                                         double>> &end_conditions,
                                             size_t order,
                                             std::vector<std::shared_ptr<Polynomial>> *ptr_traj_vec);
};

}
#endif