#include <ros/ros.h>
#include <mpdm_planner/mpdm_planner.hpp>
#include "behaviour_planner.hpp"
int main(int argc, char** argv){
  ros::init(argc, argv, "behaviour_planning_node");
  ros::NodeHandle nh;
  auto planner = std::make_unique<planning::BehaviourPlanner>();
//  planner->Launch();
  return 0;
}