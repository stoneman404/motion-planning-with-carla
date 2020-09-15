cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        carla_msgs
        derived_object_msgs
        geometry_msgs
        sensor_msgs
        planning_msgs
        planning_srvs
        tf
        std_msgs
        visualization_msgs
        carla_waypoint_types
        )
find_package(Eigen3 REQUIRED)

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
set(QPOASES_LIBRARY_DIRS /usr/local/lib)
set(IPOPT_LIBRARY_DIRS /usr/local/lib)
set(IPOPT_INCLUDE_DIRS /usr/local/include/coin)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   carla_msgs#   derived_object_msgs#   geometry_msgs#   sensor_msgs#   std_msgs#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #        INCLUDE_DIRS include
        #        LIBRARIES local_planner
        #        CATKIN_DEPENDS
        #        carla_msgs
        #        derived_object_msgs
        #        geometry_msgs
        #        sensor_msgs
        #        std_msgs
        #        visualization_msgs
        #        planning_msgs
        #        roscpp
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include/
        common/include/
        /usr/include/include/
        ${catkin_INCLUDE_DIRS}
        ${QPOASES_INCLUDE_DIRS}
        ${IPOPT_INCLUDE_DIRS}
)

#link_directories(/usr/local/lib)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/local_planner.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
set(planning_SRC
        common/src/math_utils.cpp
        common/src/box2d.cpp
        common/src/coordinate_transformer.cpp
        common/src/qunitic_polynomial.cpp
        common/src/quartic_polynomial.cpp
        common/src/polynomial.cpp

        src/maneuver_manage/maneuver_planner.cpp
        src/maneuver_manage/stop_at_sign.cpp
        src/maneuver_manage/emergency_stop_state.cpp
        src/maneuver_manage/keep_lane_state.cpp
        src/maneuver_manage/change_lane_left_state.cpp
        src/maneuver_manage/change_lane_right_state.cpp
        src/planner/frenet_lattice_planner/frenet_lattice_planner.cpp

        src/vehicle_state/vehicle_state.cpp
        src/obstacle_filter/obstacle_filter.cpp
        src/obstacle_filter/obstacle.cpp
        src/planning_config.cpp
        src/reference_line/reference_line.cpp
        src/reference_line/reference_point.cpp
        src/reference_line/reference_line_smoother.cpp
        src/planning_context.cpp
        src/planner.cpp
        src/reference_line/reference_line_smooth_ipopt_interface.cpp
        common/src/spline2d.cpp src/traffic_lights/traffic_light.cpp src/traffic_lights/traffic_light_list.cpp)

add_executable(${PROJECT_NAME}_node
        ${planning_SRC}
        src/planning_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ipopt
        qpOASES
        )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(reference_line_smoother_test

        common/src/math_utils.cpp
        common/src/box2d.cpp
        common/src/coordinate_transformer.cpp
        common/src/qunitic_polynomial.cpp
        common/src/quartic_polynomial.cpp
        common/src/polynomial.cpp

        src/vehicle_state/vehicle_state.cpp
        src/obstacle_filter/obstacle_filter.cpp
        src/obstacle_filter/obstacle.cpp
        src/planning_config.cpp
        src/reference_line/reference_line.cpp
        src/reference_line/reference_point.cpp
        src/reference_line/reference_line_smoother.cpp
        src/planning_context.cpp
        src/planning_config.cpp
        src/reference_line/reference_line.cpp
        src/reference_line/reference_line_smooth_ipopt_interface.cpp
        src/reference_line/reference_point.cpp
        src/reference_line/reference_line_smoother_test.cpp)
if (TARGET reference_line_smoother_test)
    target_link_libraries(reference_line_smoother_test
            Eigen3::Eigen
            ${catkin_LIBRARIES}
            ipopt)
endif ()

catkin_add_gtest(spline2d_test
        common/src/math_utils.cpp
        common/src/spline2d.cpp
        common/src/spline2d_test.cpp
        )

if (TARGET spline2d_test)
    target_link_libraries(spline2d_test
            Eigen3::Eigen
            ${catkin_LIBRARIES}
            )
endif ()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
