#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_FRENET_LATTICE_PLANNER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_FRENET_LATTICE_PLANNER_HPP_
#include "planner/trajectory_planner.hpp"
#include <planning_msgs/TrajectoryPoint.h>
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
   *
   * @param init_trajectory_point
   * @param reference_lines
   * @param maneuver_goal
   * @param pub_trajectory
   * @return
   */
  bool Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
               const std::list<std::shared_ptr<ReferenceLine>> &reference_lines,
               const ManeuverGoal &maneuver_goal,
               planning_msgs::Trajectory::ConstPtr pub_trajectory) override;

 protected:

  /**
   *
   * @param init_trajectory_point
   * @param ptr_ref_line
   * @param maneuver_goal
   * @param pub_trajectory
   * @return
   */
  bool Plan(const planning_msgs::TrajectoryPoint &init_trajectory_point,
            const std::shared_ptr<ReferenceLine> ptr_ref_line,
            const ManeuverGoal &maneuver_goal,
            planning_msgs::Trajectory::ConstPtr pub_trajectory) const;

  /**
   *
   * @param maneuver_goal
   * @param ptr_lon_traj_vec
   * @param ptr_lat_traj_vec
   */
  void GenerateTrajectories(const ManeuverGoal &maneuver_goal,
                            std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec,
                            std::vector<std::shared_ptr<Polynomial>> *ptr_lat_traj_vec) const;
  /**
   *
   * @param ptr_ref_line
   * @param lon_traj_vec
   * @param lat_traj_vec
   * @param ptr_combined_pub_traj
   * @return
   */
  bool CombineTrajectories(const std::shared_ptr<ReferenceLine> ptr_ref_line,
                           const Polynomial &lon_traj_vec, const Polynomial &lat_traj_vec,
                           std::shared_ptr<planning_msgs::Trajectory> ptr_combined_pub_traj);

 private:

  /**
   *
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
   *
   * @param cruise_speed
   * @param ptr_lon_traj_vec
   */
  void GenerateCruisingLonTrajectories(const double cruise_speed,
                                       std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const;

  /**
   *
   * @param stop_s
   * @param ptr_lon_traj_vec
   */
  void GenerateStoppingLonTrajectories(const double stop_s,
                                       std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const;

  /**
   *
   * @param ptr_lon_traj_vec
   */
  void GenerateOvertakeAndFollowingLonTrajectories(std::vector<std::shared_ptr<Polynomial>> *ptr_lon_traj_vec) const;

};

}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_PLANNER_FRENET_LATTICE_PLANNER_HPP_