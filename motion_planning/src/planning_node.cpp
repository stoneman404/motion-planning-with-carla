#include <ros/ros.h>
#include "planner.hpp"
#include <memory>
int main(int argc, char **argv) {
  ros::init(argc, argv, "planning_node");
  ros::NodeHandle nh;
  auto planner = std::make_unique<planning::Planner>(nh);
  ros::Rate loop_rate(10);

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}