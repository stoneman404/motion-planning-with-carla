#include <ros/ros.h>
#include "planner.hpp"
#include <memory>
int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_planning_node");
  ros::NodeHandle nh;
  auto planner = std::make_unique<planning::Planner>(nh);
  ros::Rate loop_rate(10);

  while(ros::ok()) {
    ROS_DEBUG("Planning_node runonce");
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}