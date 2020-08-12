
#ifndef CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_VEHICLE_STATE_VEHICLE_STATE_HPP_
#define CATKIN_WS_SRC_LOCAL_PLANNER_INCLUDE_VEHICLE_STATE_VEHICLE_STATE_HPP_
#include <ros/ros.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>

namespace planning{
class VehicleState{
public:

    static planning::VehicleState& Instance();
    bool Update(const carla_msgs::CarlaEgoVehicleStatus& ego_vehicle_status,
        const nav_msgs::Odometry& odometry,
        const carla_msgs::CarlaEgoVehicleInfo& vehicle_info);
    // getter
    const ros::Time& time_stamp() const;
    const geometry_msgs::Pose& pose() const;
    double kappa() const;
    double heading() const;
    double linear_vel() const;
    double angular_vel() const;
    double linear_acc() const;
    double centripential_acc() const{ return centripental_acc_;}
    double steer_percentage() const {return steer_percentage_;}
    const geometry_msgs::Vector3 center_of_mass() const {return center_of_mass_;}

    // setter
    void set_linear_vel(double vel) ;
    void set_angular_vel(double omega);
    void set_pose(const geometry_msgs::Pose& pose);
    void set_linear_acc(double acc);
    void set_center_of_mass(const geometry_msgs::Vector3& center_of_mass) {this->center_of_mass_ = center_of_mass;}
    // predict the vehicle's last pose based on current pose, linear/angular acc and vel after time t;
    geometry_msgs::Pose PredictNextPose(double t);

private:
    geometry_msgs::Pose pose_;
    double kappa_;
    double heading_;

    double linear_vel_;
    double angular_vel_;
    double linear_acc_;
    double centripental_acc_;
    double steer_percentage_;
    bool reverse_ = false;
    geometry_msgs::Vector3 center_of_mass_;
    ros::Time time_stamp_;

private:
    VehicleState() = default;
    ~VehicleState() = default;
};
}

#endif
