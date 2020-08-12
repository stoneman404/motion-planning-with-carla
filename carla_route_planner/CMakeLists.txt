cmake_minimum_required(VERSION 2.8.3)
project(carla_route_planner)

find_package(catkin REQUIRED COMPONENTS rospy carla_waypoint_types roslaunch planning_msgs planning_srvs)

catkin_python_setup()

roslaunch_add_file_check(launch)

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(
  PROGRAMS src/carla_route_planner/carla_route_planner.py DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
