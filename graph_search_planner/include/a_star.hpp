#ifndef PLANNING_ALGORITHM_SRC_GRAPH_SEARCH_PLANNER_INCLUDE_A_STAR_HPP_
#define PLANNING_ALGORITHM_SRC_GRAPH_SEARCH_PLANNER_INCLUDE_A_STAR_HPP_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include "graph_search_utils.hpp"
#include <map>

namespace planning {
class Node2d {
public:
    Node2d(
        double x, double y,
        nav_msgs::OccupancyGrid::Ptr grid_map) {
        Pose2Index(x,y, grid_map, &index_x_, &index_y_);
        index_ = std::to_string(index_x_) + "_" + std::to_string(index_y_);
    }
    Node2d(int x, int y) {
        index_x_ = x;
        index_y_ = y;
        index_ = std::to_string(index_x_) + "_" + std::to_string(index_y_);
    }
    //////////////////// setter /////////////////////////////
    void SetHCost(double h) {
        hcost_ = h;
        fcost_ = hcost_ + gcost_;
    }
    void SetFcost(double f) { fcost_ = f; }
    void SetGcost(double g) {
        gcost_ = g;
        fcost_ = gcost_ + hcost_;
    }
    void SetPreNode(std::shared_ptr<Node2d> pre_node) {
        pre_node_ = pre_node;
    }

    /////////////////// getter ///////////////////////
    double GetHCost() const { return hcost_; }
    double GetFCost() const { return fcost_; }
    double GetGCost() const { return gcost_; }
    int GetIndexX() const { return index_x_; }
    int GetIndexY() const { return index_y_; }
    const std::string &GetIndex() const { return index_; }
    std::shared_ptr<Node2d> GetPreNode() const { return pre_node_; }
    /////////////// operator ////////////////////////
    bool operator==(const Node2d &other) const {
        return other.GetIndex() == index_;
    }
protected:

private:
    double gcost_ = 0.0;
    double fcost_ = 0.0;
    double hcost_ = 0.0;
    int index_x_ = 0;
    int index_y_ = 0;
    std::string index_ = "";
    std::shared_ptr<Node2d> pre_node_ = nullptr;
};

struct AStarResult {
  std::vector<double> x;
  std::vector<double> y;
  double cost = 0.0;
};

class AStar {
public:

    AStar() = default;
    ~AStar() = default;
    void SetVhicleParams(double length, double width, double axle_ref_x){
        length_= length;
        width_ = width;
        axle_ref_x_ = axle_ref_x_;
    }
    bool SearchPath(double sx, double sy, double ex, double ey, AStarResult *result);

    void SetMap(nav_msgs::OccupancyGrid::Ptr grid_map) {
        grid_map_ = std::move(grid_map);
        has_map_ = true;
        xy_resolution_ = grid_map_->info.resolution;
    }
    /**
     * check the dp map to get the cost
     * @param sx: the curruent position x
     * @param sy: the current position y
     * @return
     */
    double CheckDpMap(double sx, double sy);
    /**
     *
     * @param ex: the end position x
     * @param ey: the end position y
     * @return true or false
     */
    bool GenerateDpMap(double ex, double ey);

protected:

    /**
     * Euclid Distance, as heuristic
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return
     */
    double EuclidDistance(std::shared_ptr<Node2d> cur, std::shared_ptr<Node2d> next) {
        double h;
        const int cur_index_x = cur->GetIndexX();
        const int cur_index_y = cur->GetIndexY();
        const int next_index_x = next->GetIndexX();
        const int next_index_y = next->GetIndexY();
        double cur_x, cur_y, next_x, next_y;
        Index2Pose(cur_index_x, cur_index_y, grid_map_, &cur_x, &cur_y);
        Index2Pose(next_index_x, next_index_y, grid_map_, &next_x, &next_y);
        h = std::hypot(next_x - cur_x, next_y - cur_y);
        double tie_breaker = 1;
        return tie_breaker * h;
    }

    double MahDistance(std::shared_ptr<Node2d> cur, std::shared_ptr<Node2d> next){
        double h;
        const int cur_index_x = cur->GetIndexX();
        const int cur_index_y = cur->GetIndexY();
        const int next_index_x = next->GetIndexX();
        const int next_index_y = next->GetIndexY();
        double cur_x, cur_y, next_x, next_y;
        Index2Pose(cur_index_x, cur_index_y, grid_map_, &cur_x, &cur_y);
        Index2Pose(next_index_x, next_index_y, grid_map_, &next_x, &next_y);
        h = std::fabs(next_x - cur_x) + std::fabs(next_y - cur_y);
        return h;
    }

    double EdgeCost(std::shared_ptr<Node2d> cur, std::shared_ptr<Node2d> next){
        double cost;
        const int cur_index_x = cur->GetIndexX();
        const int cur_index_y = cur->GetIndexY();
        const int next_index_x = next->GetIndexX();
        const int next_index_y = next->GetIndexY();
        double cur_x, cur_y, next_x, next_y;
        Index2Pose(cur_index_x, cur_index_y, grid_map_, &cur_x, &cur_y);
        Index2Pose(next_index_x, next_index_y, grid_map_, &next_x, &next_y);
        cost = std::hypot(next_x - cur_x, next_y - cur_y);
        return cost;
    }

    bool VerifyNode2d(std::shared_ptr<Node2d> node);

    std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(std::shared_ptr<Node2d> node);

    bool TracePath(AStarResult *astar_path);


    struct cmp {
      bool operator()(const std::shared_ptr<Node2d> left, const std::shared_ptr<Node2d> right) const{
          return left->GetFCost() >= right->GetFCost();
      }
    };

private:
    double length_;
    double width_;
    double axle_ref_x_;
    double xy_resolution_;
    double node_radius_;
    std::shared_ptr<Node2d> start_node_ = nullptr;
    std::shared_ptr<Node2d> goal_node_ = nullptr;
    std::shared_ptr<Node2d> final_node_ = nullptr;
    std::unordered_map<std::string, std::shared_ptr<Node2d>> dp_map_;
    bool has_map_ = false;
    nav_msgs::OccupancyGrid::Ptr grid_map_;

};

}
#endif //PLANNING_ALGORITHM_SRC_GRAPH_SEARCH_PLANNER_INCLUDE_A_STAR_HPP_
