#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_POINT_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_POINT_HPP_
#include <Eigen/Core>
namespace planning {
class ReferencePoint {
 public:
  ReferencePoint() = default;
  ~ReferencePoint() = default;
  explicit ReferencePoint(Eigen::Vector2d xy,
                          const double &heading = 0, const double &kappa = 0,
                          const double &dkappa = 0, const double &ddkappa = 0);
  ReferencePoint(const double &x, const double &y,
                 const double &heading = 0, const double &kappa = 0,
                 const double &dkappa = 0, const double &ddkappa = 0);
  ReferencePoint(const ReferencePoint &other);

  ///////// getter ////////////////
  const Eigen::Vector2d &xy() const;
  const double &x() const;
  const double &y() const;
  const double &theta() const;
  const double &kappa() const;
  const double &dkappa() const;
  const double &ddkappa() const;
  ////////////// setter /////////////
  void set_xy(const Eigen::Vector2d &xy);
  void set_xy(const double &x, const double &y);
  void set_theta(const double &heading);
  void set_kappa(const double &kappa);
  void set_dkappa(const double &dkappa);
  void set_ddkappa(const double &ddkappa);
 private:
  Eigen::Vector2d xy_;
  double theta_{};
  double kappa_{};
  double dkappa_{};
  double ddkappa_{};
};
}
#endif //
