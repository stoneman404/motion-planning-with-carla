//
// Created by ldh on 20-7-17.
//
#include "graph_search_utils.hpp"
namespace planning{
bool InBoudary(int index_x, int index_y,  nav_msgs::OccupancyGrid::Ptr grid_map) {
    return index_x >= 0 && index_x < grid_map->info.width && index_y >= 0
        && index_y < grid_map->info.height;
}

void Index2Pose(int index_x, int index_y,
                 nav_msgs::OccupancyGrid::Ptr grid_map,
                double *x, double *y) {
    const double theta = tf::getYaw(grid_map->info.origin.orientation);
    const double robot_x = index_x * grid_map->info.resolution;
    const double robot_y = index_y * grid_map->info.resolution;
//    const double relative_x = std::cos(theta) * robot_x - std::sin(theta) * robot_y;
//    const double relative_y = std::sin(theta) * robot_x + std::cos(theta) * robot_y;
    const double relative_x = robot_x;
    const double relative_y = robot_y;
    *x = grid_map->info.origin.position.x + relative_x;
    *y = grid_map->info.origin.position.y + relative_y;
}

void Pose2Index(double x, double y,
    nav_msgs::OccupancyGrid::Ptr grid_map,
                int *index_x, int *index_y) {
//    double origin_x = grid_map->info.origin.position.x;
//    double origin_y = grid_map->info.origin.position.y;
//    double origin_theta = tf::getYaw(grid_map->info.origin.orientation);
//    double dx = x - origin_x;
//    double dy = y - origin_y;
//    const double robot_x = std::cos(origin_theta) * dx + std::sin(origin_theta) * dy;
//    const double robot_y = -std::sin(origin_theta) * dx + std::cos(origin_theta) * dy;
    *index_x = static_cast<int>(std::round(x / grid_map->info.resolution));
    *index_y = static_cast<int>(std::round(y / grid_map->info.resolution));
}

int CalcIndex(int index_x, int index_y, int width, int height) {
    int index = index_y * width + index_x;
    return index;
}

bool CheckPose2d(double x, double y,  nav_msgs::OccupancyGrid::Ptr grid_map) {
    int index_x, index_y;
    Pose2Index(x, y, grid_map, &index_x, &index_y);
    if (!InBoudary(index_x, index_y, grid_map)) {
        return false;
    }
    int index = CalcIndex(index_x, index_y, grid_map->info.width, grid_map->info.height);
    return grid_map->data[index] < 0.1;
}

double NormalizeAngle(const double &angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

double CalcAngleDist(double from, double to) {
    return NormalizeAngle(to - from);
}


bool CheckPose3d(double base_x, double base_y, double base_phi,

                 double adc_length, double adc_width,
                 double axle_ref_x, double expand_dist,
                 nav_msgs::OccupancyGrid::Ptr grid_map) {

    const double adc_expand_length = adc_length + expand_dist;
    const double adc_expand_width = adc_width + expand_dist;
    const double axle2back = 0.5 * adc_expand_length - axle_ref_x;
    if (axle2back < 0.0) {
        return false;
    }
    const double left = -1.0 * axle2back;
    const double right = adc_expand_length - 1.0 * axle2back;
    const double top = adc_expand_width / 2.0;
    const double bottom = -adc_expand_width / 2.0;
    const double sin_phi = std::sin(base_phi);
    const double cos_phi = std::cos(base_phi);
    double xy_resolution = std::min(0.5, static_cast<double>(grid_map->info.resolution));
    for (double x = left; x < right; x += xy_resolution) {
        for (double y = top; y > bottom; y -= xy_resolution) {
            double map_x = x * cos_phi - y * sin_phi + base_x;
            double map_y = x * sin_phi + y * cos_phi + base_y;
            if (!CheckPose2d(map_x, map_y, grid_map)) {
                return false;
            }
        }
    }
//    ROS_INFO("CheckPose3d End");
    return true;
}

bool CheckCircleRobotPose(double x, double y, double raidus,
                          nav_msgs::OccupancyGrid::Ptr grid_map){
    for (double theta = -M_PI; theta < M_PI; theta += 0.2){
        for (double r = 0.0; r < raidus; r += 0.2){
            double robot_x = x + r * std::cos(theta);
            double robot_y = y + r * std::sin(theta);
            if (!CheckPose2d(robot_x, robot_y, grid_map)){
                return false;
            }
        }
    }
    return true;
}

}

