//
// Created by ldh on 20-7-14.
//
#include <ros/ros.h>
#include <hybrid_a_star.hpp>

#include <memory>

int main(int argc, char** argv) {
    ros::init(argc, argv, "hybrid_a_star_planner");
    ros::NodeHandle nh;
    std::unique_ptr<planning::HybridAStar> hybrid_a_star =
        std::make_unique<planning::HybridAStar>(nh);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        ros::Time begin = ros::Time::now();
        hybrid_a_star->RunOnce();
        ros::Time end = ros::Time::now();
        ros::Duration d(end - begin);
        ROS_WARN("[RUNONCE Time: %lf ms]", d.toNSec() / 1000000.0);
    }
    return 0;
}