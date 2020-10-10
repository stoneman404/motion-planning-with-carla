#include "motion_planner/frenet_lattice_planner/trajectory_selector.hpp"

namespace planning{

TrajectorySelector::TrajectorySelector(const std::array<double, 3> &init_s,
                                       const std::vector<std::shared_ptr<Polynomial>> &lon_trajectory_vec,
                                       const std::vector<std::shared_ptr<Polynomial>> &lat_trajectory_vec,
                                       std::shared_ptr<ReferenceLine> ptr_ref_line,
                                       std::shared_ptr<STGraph> ptr_st_graph) {

}
}