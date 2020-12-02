#include <ros/ros.h>
#include "behaviour_planner.hpp"

int main(int argc, char** argv){
//  std::cout<<"hello world" << std::endl;
  ros::init(argc, argv, "behaviour_planning_node");
  ros::NodeHandle nh;
  auto behaviour_planner = std::make_unique<planning::BehaviourPlanner>(nh);
  ros::WallRate loop_rate(10.0);
  while(ros::ok()){
    ros::spinOnce();
    behaviour_planner->RunOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
//  planner->Launch();
  return 0;
}

