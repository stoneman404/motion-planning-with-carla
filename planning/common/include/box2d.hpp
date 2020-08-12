
#ifndef CATKIN_WS_SRC_PLANNING_MATH_INCLUDE_BOX2D_HPP_
#define CATKIN_WS_SRC_PLANNING_MATH_INCLUDE_BOX2D_HPP_
//from apollo
#include <Eigen/Core>
#include <vector>
namespace planning{
class Box2d{
public:
    Box2d() = default;
    ~Box2d() = default;
    Box2d(const Eigen::Vector2d& center, double heading, double length, double width);
    const Eigen::Vector2d& Center() const { return center_;}
    double center_x() const {return center_.x();}
    double center_y() const {return center_.y();}
    double length() const {return length_;}
    double width() const {return width_;}
    double half_length() const {return half_length_;}
    double half_width() const {return half_width_;}
    double heading() const {return heading_;}
    double cos_heading() const {return cos_heading_;}
    double sin_heading() const {return sin_heading_;}
    double area() const {return length_ * width_;}
    // the diagonal length
    double diagonal() const {return std::hypot(length_, width_);}
    std::vector<Eigen::Vector2d> GetAllCorners() const;
    bool IsPointIn(const Eigen::Vector2d& point) const;
    bool IsPointOnBoundary(const Eigen::Vector2d& point) const;
    double DistanceToPoint(const Eigen::Vector2d& poiny) const;
    bool HasOverlapWithBox2d(const Box2d& box) const;
    void RotateFromCenter(const double rotate_angle);
    void Shift(const Eigen::Vector2d& shift_vec);
    void LongitudinalExtend(double extension_length);
    void LateralExtend(double extension_length);
    void InitCorners();
    double max_x() const {return max_x_;}
    double min_x() const {return min_x_;}
    double max_y() const {return max_x_;}
    double min_y() const {return min_y_;}


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

#endif //
