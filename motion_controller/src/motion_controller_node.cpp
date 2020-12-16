#include <ros/ros.h>
#include "controller.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "motion_controller_node");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
  auto controller = std::make_unique<control::Controller>(nh);
  controller->Launch();
  return 0;
}