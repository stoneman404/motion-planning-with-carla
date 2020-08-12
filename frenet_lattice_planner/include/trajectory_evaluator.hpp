
#ifndef PLANNING_ALGORITHM_SRC_FRENET_LATTICE_PLANNER_INCLUDE_TRAJECTORY_EVALUATOR_HPP_
#define PLANNING_ALGORITHM_SRC_FRENET_LATTICE_PLANNER_INCLUDE_TRAJECTORY_EVALUATOR_HPP_
#include <ros/ros.h>
namespace planning{
namespace frenet_lattice{
class TrajectoryEvaluator{
public:
    TrajectoryEvaluator();
    ~TrajectoryEvaluator();
    bool Evaluate();
private:

};
}

}
#endif //PLANNING_ALGORITHM_SRC_FRENET_LATTICE_PLANNER_INCLUDE_TRAJECTORY_EVALUATOR_HPP_
