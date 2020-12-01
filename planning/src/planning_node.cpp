#include <ros/ros.h>
#include "planner.hpp"

int main(int argc, char** argv){
//  std::cout<<"hello world" << std::endl;
  ros::init(argc, argv, "planning_node");
  ros::NodeHandle nh;
  auto planner = std::make_unique<planning::Planner>(nh);
//  planner->Launch();
  return 0;
}

