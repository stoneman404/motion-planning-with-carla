#include <ros/ros.h>
#include "controller.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "behaviour_planning_node");
  ros::NodeHandle nh;
  auto controller = std::make_unique<control::Controller>(nh);
  controller->Launch();
  return 0;
}