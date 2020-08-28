#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTH_IPOPT_INTERFACE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTH_IPOPT_INTERFACE_HPP_
#include <cppad/ipopt/solve.hpp>
#include <string>
#include "reference_line.hpp"
#include "planning_config.hpp"


namespace planning{
class ReferenceLineSmoothIpoptInterface{
public:

    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    explicit ReferenceLineSmoothIpoptInterface(const std::vector<ReferencePoint>& ref_points);
    ~ReferenceLineSmoothIpoptInterface() = default;
    void operator()(ADvector& fg, const ADvector& x);
    void set_ref_deviation_weight(double weight) {this->ref_deviation_weight_ = weight;}
    void set_curvature_weight(double weight) {this->curvature_weight_ = weight;}
    void set_length_weight(double weight) {this->length_weight_ = weight;}
    void set_heading_weight(double weight) {this->heading_weight_ = weight;}

private:
    std::vector<ReferencePoint> ref_points_;
    size_t number_of_points_;
    size_t number_of_variables_;
    size_t number_of_curvature_constraints_;
    size_t number_of_constraints_;
    double ref_deviation_weight_ ;
    double curvature_weight_ ;
    double length_weight_ ;
    double heading_weight_;
    double max_curvature_;


};


}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_SMOOTH_IPOPT_INTERFACE_HPP_
