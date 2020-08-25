#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_
#include "reference_line.hpp"
#include "planning_config.hpp"
#include "math_utils.hpp"
#include <cppad/ipopt/solve.hpp>
#include <glog/logging.h>
namespace planning {
class ReferenceLineSmoother {
public:
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADVector;
    typedef CPPAD_TESTVECTOR(double) DVector;
    ReferenceLineSmoother() = default;
    ~ReferenceLineSmoother() = default;
    bool GetSmoothReferenceLine(const planning_srvs::RouteResponse& route_response,
        ReferenceLine* smoothed_ref_line);
    bool GetSmoothReferenceLine(const std::vector<planning_msgs::WayPoint>& waypoints,
        ReferenceLine* smoothed_ref_line);
    bool GetSmoothReferenceLine(const ReferenceLine &raw_ref_line,
                                ReferenceLine *smoothed_ref_line);

private:
    bool SetUpConstraint();
    void SetUpOptions();
    void SetUpInitValue();
    bool TraceSmoothReferenceLine(
        const CppAD::ipopt::solve_result<DVector> &result,
        ReferenceLine *smoothed_ref_line);

private:
    ReferenceLine raw_ref_line_;
    std::string options_;
    DVector x_l_;
    DVector x_u_;
    DVector g_l_;
    DVector g_u_;
    DVector xi_;

};

}

#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_
