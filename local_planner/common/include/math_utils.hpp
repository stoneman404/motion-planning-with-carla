#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MATH_UTILS_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MATH_UTILS_HPP_

#include <Eigen/Core>
#include <vector>
#include <planning_msgs/Trajectory.h>
namespace planning {

double NormalizeAngle(const double &angle);
double CalcAngleDist(double from, double to);
planning_msgs::TrajectoryPoint InterpolateTrajectoryPoint(
    const planning_msgs::TrajectoryPoint &p0,
    const planning_msgs::TrajectoryPoint &p1, double time);


double CalcKappa();
double CalcDKappa();
double CalcDDKappa();

double slerp(double a0, double t0, double a1, double t1,
             double t);
double lerp(const double &x0, const double &t0,
            const double &x1, const double &t1, const double& t);

}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MATH_UTILS_HPP_