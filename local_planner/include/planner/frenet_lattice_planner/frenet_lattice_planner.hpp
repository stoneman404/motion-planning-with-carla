#include "planner/trajectory_planner.hpp"
#include <planning_msgs/TrajectoryPoint.h>
namespace planning {

class FrenetLatticePlanner : public TrajectoryPlanner {
 public:
  FrenetLatticePlanner() = default;
  ~FrenetLatticePlanner() override = default;
  bool Process(const planning_msgs::TrajectoryPoint &init_trajectory_point,
               const std::list<std::shared_ptr<ReferenceLine>> &reference_lines,
               planning_msgs::Trajectory::ConstPtr pub_trajectory) override;
};

}