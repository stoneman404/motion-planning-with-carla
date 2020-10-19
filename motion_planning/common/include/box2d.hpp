
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_BOX2D_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_BOX2D_HPP_
//from apollo
#include <Eigen/Core>
#include <vector>
namespace planning{
class Box2d{
public:
    Box2d() = default;
    ~Box2d() = default;
    Box2d(const Eigen::Vector2d& center, double heading, double length, double width);
    const Eigen::Vector2d& Center() const;
    double center_x() const;
    double center_y() const;
    double length() const;
    double width() const;
    double half_length() const;
    double half_width() const;
    double heading() const;
    double cos_heading() const;
    double sin_heading() const;
    double area() const;
    // the diagonal length
    double diagonal() const;
    std::vector<Eigen::Vector2d> GetAllCorners() const;
    bool IsPointIn(const Eigen::Vector2d& point) const;
    bool IsPointOnBoundary(const Eigen::Vector2d& point) const;
    double DistanceToPoint(const Eigen::Vector2d& poiny) const;
    bool HasOverlapWithBox2d(const Box2d& box) const;
    void RotateFromCenter(double rotate_angle);
    void Shift(const Eigen::Vector2d& shift_vec);
    void LongitudinalExtend(double extension_length);
    void LateralExtend(double extension_length);
    void InitCorners();
    double max_x() const;
    double min_x() const;
    double max_y() const;
    double min_y() const;


private:
    Eigen::Vector2d center_;
    double length_ = 0.0;
    double width_ = 0.0;
    double half_length_ = 0.0;
    double half_width_ = 0.0;
    double heading_ = 0.0;
    double cos_heading_ = 1.0;
    double sin_heading_ = 0.0;
    std::vector<Eigen::Vector2d> corners_;
    double max_x_ = std::numeric_limits<double>::lowest();
    double min_x_ = std::numeric_limits<double>::max();
    double max_y_ = std::numeric_limits<double>::lowest();
    double min_y_ = std::numeric_limits<double>::max();
};
}

#endif //CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_BOX2D_HPP_
