cmake_minimum_required(VERSION 3.0.2)
project(common)

add_compile_options(-std=c++14 -g -Werror -Wall -Wno-unused -Wno-sign-compare)


find_package(catkin REQUIRED COMPONENTS
        planning_msgs
        planning_srvs
        roscpp
        rospy
        derived_object_msgs
        sensor_msgs
        carla_waypoint_types
        carla_msgs
        geometry_msgs
        std_msgs
        nav_msgs
        tf
        )
find_package(Eigen3 REQUIRED)


catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        LIBRARIES common
        #        CATKIN_DEPENDS planning_msgs planning_srvs roscpp rospy derived_object_msgs sensor_msgs carla_waypoint_types carla_msgs geometry_msgs std_msgs nav_msgs tf
        #        DEPENDS Eigen3
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${IPOPT_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(common
        src/math/frenet_frame.cpp
        src/math/math_utils.cpp
        src/math/coordinate_transformer.cpp
        src/math/math_utils.cpp
        src/polygon/box2d.cpp
        src/curves/simple_spline.cpp
        src/curves/qunitic_polynomial.cpp
        src/curves/quartic_polynomial.cpp
        src/curves/spline2d.cpp
        )

target_link_libraries(common
        ${catkin_LIBRARIES}
        ${Eigen3_LIBRARIES}
        )

add_dependencies(common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_dependencies(collision_checker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
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
#install(TARGETS common
#        #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        #   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#        )
#install(TARGETS reference_line
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#
#install(TARGETS vehicle_state
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#
#install(TARGETS obstacle
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#
#install(TARGETS collision_checker
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#### Mark cpp header files for installation
# install(DIRECTORY include/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
##   FILES_MATCHING PATTERN "*.h"
##   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_common.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

catkin_add_gtest(box2d_test
        src/polygon/box2d.cpp
        src/polygon/box2d_test.cpp
        src/math/math_utils.cpp)
if (TARGET box2d_test)
    target_link_libraries(box2d_test
            ${catkin_LIBRARIES}
            ${Eigen3_LIBRARIES})
endif ()

catkin_add_gtest(spline_test
        src/curves/spline2d_test.cpp
        src/math/math_utils.cpp
        src/curves/simple_spline.cpp
        src/curves/spline2d.cpp
        )
if (TARGET spline_test)
    target_link_libraries(box2d_test
            ${catkin_LIBRARIES}
            ${Eigen3_LIBRARIES})
endif ()
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
