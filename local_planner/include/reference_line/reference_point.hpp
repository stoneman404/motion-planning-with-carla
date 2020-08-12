#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_POINT_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_REFERENCE_LINE_REFERENCE_POINT_HPP_
#include <Eigen/Core>
namespace planning {
class ReferencePoint {
public:
    ReferencePoint() = default;
    ~ReferencePoint() = default;
    explicit ReferencePoint(const Eigen::Vector2d &xy,
                   const double &heading = 0, const double &kappa = 0,
                   const double &dkappa = 0, const double &ddkappa = 0);
    ReferencePoint(const double &x, const double &y,
                   const double &heading = 0, const double &kappa = 0,
                   const double &dkappa = 0, const double &ddkappa = 0);
    ReferencePoint(const ReferencePoint &other);

    ///////// getter ////////////////
    const Eigen::Vector2d &xy() const { return xy_; }
    const double &x() const { return xy_[0]; }
    const double &y() const { return xy_[1]; }
    const double &heading() const { return heading_; }
    const double &kappa() const { return kappa_; }
    const double &dkappa() const { return dkappa_; }
    const double &ddkappa() const { return ddkappa_; }
    ////////////// setter /////////////
    void set_xy(const Eigen::Vector2d &xy) { this->xy_ = xy; }
    void set_xy(const double &x, const double &y) {
      this->xy_[0] = x;
      this->xy_[1] = y;
    }
    void set_heading(const double &heading) { this->heading_ = heading; }
    void set_kappa(const double &kappa) { this->kappa_ = kappa; }
    void set_dkappa(const double &dkappa) { this->dkappa_ = dkappa; }
    void set_ddkappa(const double &ddkappa) { this->ddkappa_ = ddkappa; }

private:
    Eigen::Vector2d xy_;
    double heading_;
    double kappa_;
    double dkappa_;
    double ddkappa_;
};
}
#endif //
