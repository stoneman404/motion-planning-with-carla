
#ifndef CATKIN_WS_SRC_GRAPH_SEARCH_PLANNER_INCLUDE_GRAPH_SEARCH_UTILS_HPP_
#define CATKIN_WS_SRC_GRAPH_SEARCH_PLANNER_INCLUDE_GRAPH_SEARCH_UTILS_HPP_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace planning {

bool InBoudary(int index_x, int index_y,
     nav_msgs::OccupancyGrid::Ptr grid_map);
void Index2Pose(int index_x, int index_y,
                 nav_msgs::OccupancyGrid::Ptr grid_map,
                double *x, double *y);
void Pose2Index(double x, double y,
                 nav_msgs::OccupancyGrid::Ptr grid_map,
                int *index_x, int *index_y);
int CalcIndex(int index_x, int index_y, int width, int height);
bool CheckPose2d(double x, double y,
     nav_msgs::OccupancyGrid::Ptr grid_map);
double NormalizeAngle(const double &angle);
double CalcAngleDist(double from, double to);
bool CheckPose3d(double base_x, double base_y, double base_phi,
                 double adc_length, double adc_width,
                 double axle_ref_x, double expand_dist,
                 nav_msgs::OccupancyGrid::Ptr grid_map);
bool CheckCircleRobotPose(double x, double y, double raidus,
    nav_msgs::OccupancyGrid::Ptr grid_map);
}

#endif //CATKIN_WS_SRC_GRAPH_SEARCH_PLANNER_INCLUDE_GRAPH_SEARCH_UTILS_HPP_
