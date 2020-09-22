#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MATH_UTILS_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MATH_UTILS_HPP_

#include <Eigen/Dense>
#include <vector>
#include <planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Pose.h>

namespace planning {
class MathUtil {
 public:
  static double NormalizeAngle(const double &angle);
  static double CalcAngleDist(double from, double to);
  static planning_msgs::TrajectoryPoint InterpolateTrajectoryPoint(
      const planning_msgs::TrajectoryPoint &p0,
      const planning_msgs::TrajectoryPoint &p1, double time);
  static Eigen::Vector3d Transform(const geometry_msgs::Pose &base_transform,
                                   const Eigen::Vector3d &in_point);

  static double CalcKappa(double dx, double dy, double ddx, double ddy);
  static double CalcDKappa(double dx, double dy, double ddx, double ddy, double dddx, double dddy);

  static double slerp(double a0, double t0, double a1, double t1,
                      double t);
  static double lerp(const double &x0, const double &t0,
                     const double &x1, const double &t1,
                     const double &t);

};

}
#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_MATH_UTILS_HPP_