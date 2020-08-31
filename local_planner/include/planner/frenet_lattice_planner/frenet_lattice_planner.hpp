#include "planner/trajectory_planner.hpp"

namespace planning{

class FrenetLatticePlanner : public TrajectoryPlanner{
public:
    FrenetLatticePlanner() = default;
    ~FrenetLatticePlanner() override = default;
    bool Execute() override ;
};

}