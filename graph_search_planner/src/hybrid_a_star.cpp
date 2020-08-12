#include <memory>
#include <cmath>
#include <hybrid_a_star.hpp>
#include <graph_search_utils.hpp>
#include <nav_msgs/Path.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <visualization_msgs/MarkerArray.h>

namespace planning {

Node3d::Node3d(double x, double y, double phi) {
    x_ = x;
    y_ = y;
    phi_ = phi;
}

Node3d::Node3d(double x, double y, double phi,
               nav_msgs::OccupancyGrid::Ptr grid_map,
               double phi_resolution) {
    x_ = x;
    y_ = y;
    phi_ = phi;
    Pose2Index(x, y, grid_map, &index_x_, &index_y_);
    index_phi_ = static_cast<int>((phi_ - (-M_PI)) / phi_resolution);
    index_ = std::to_string(index_x_) + "_" + std::to_string(index_y_) + "_" + std::to_string(index_phi_);
    traversed_x_.push_back(x);
    traversed_y_.push_back(y);
    traversed_phi_.push_back(phi);
    step_size_ = traversed_x_.size();

}

Node3d::Node3d(const std::vector<double> &traversed_x,
               const std::vector<double> &traversed_y,
               const std::vector<double> &traversed_phi,
               nav_msgs::OccupancyGrid::Ptr grid_map,
               double phi_resolution) {
    ROS_ASSERT(traversed_x.size() == traversed_y.size());
    ROS_ASSERT(traversed_x.size() == traversed_phi.size());
    traversed_x_ = std::move(traversed_x);
    traversed_y_ = std::move(traversed_y);
    traversed_phi_ = std::move(traversed_phi);
    x_ = traversed_x_.back();
    y_ = traversed_y_.back();
    phi_ = traversed_phi_.back();
    Pose2Index(x_, y_, grid_map, &index_x_, &index_y_);
    index_phi_ = static_cast<int>((phi_ - (-M_PI)) / phi_resolution);
    index_ = std::to_string(index_x_) + "_"
        + std::to_string(index_y_) + "_" + std::to_string(index_phi_);
    step_size_ = traversed_x_.size();
}

bool Node3d::operator==(const planning::Node3d &other) const {
    return other.GetIndex() == index_;
}

/////////////////////////// hybrid a star class ////////////////
HybridAStar::HybridAStar(const ros::NodeHandle &nh) {
    nh_ = nh;
    ha_star_path_publisher_ = nh_.advertise<nav_msgs::Path>("/ha_path", 1);
    visualized_vehicle_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualized_vehicle", 1);
    visulized_path_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualized_path", 1);
    grid_map_subscriber_ = nh_.subscribe("/map", 10, &HybridAStar::SetMap, this);
    init_pose_subscriber_ = nh_.subscribe("/initialpose", 10, &HybridAStar::SetStart, this);
    goal_pose_subscriber_ = nh_.subscribe("/move_base_simple/goal", 10, &HybridAStar::SetGoal, this);
    nh_.param<int>("/next_node_num", next_node_num_, 10);
    nh_.param<double>("/max_steer_angle", max_steer_angle_, M_PI / 6.0);
    nh_.param<double>("/adc_width", adc_width_, 1.1);
    nh_.param<double>("/adc_length", adc_length_, 2);
    nh_.param<double>("/axle_ref_x", axle_ref_x_, 0.3);
    nh_.param<double>("/phi_resolution", phi_resolution_, 0.1);
    nh_.param<double>("/traj_forward_penalty", traj_forward_penalty_, 1.0);
    nh_.param<double>("/traj_backward_penalty", traj_backward_penalty_, 2.0);
    nh_.param<double>("/traj_gear_switch_penalty", traj_gear_switch_penalty_, 1.5);
    nh_.param<double>("/traj_steer_penalty", traj_steer_penalty_, 1.15);
    nh_.param<double>("/traj_steer_change_penalty", traj_steer_change_penalty_, 1.05);
    nh_.param<double>("/traj_step_length_penalty", traj_step_length_penalty_, 1.0);
    nh_.param<double>("/step_size", step_size_, 0.7);
    nh_.param<double>("/wheel_base", wheel_base_, 1.2);
    nh_.param<bool>("/enable_backward", enable_backward_, true);
    nh_.param<double>("/delta_t", delta_t_, 0.4);
    nh_.param<double>("/anlytic_expand_distance", anlytic_expand_distance_, 10);
    heuristic_generator_ = std::make_unique<AStar>();
}

std::unordered_map<std::string, bool> HybridAStar::GenerateCollisionCheckLookUpTable() {
    int phi_size = std::lround(2.0 * M_PI / phi_resolution_);

    std::unordered_map<std::string, bool> collision_lookup_table;
    for (int ix = 0; ix < grid_map_->info.width; ++ix) {
        for (int iy = 0; iy < grid_map_->info.height; ++iy) {
            for (int itheta = 0; itheta < phi_size; ++itheta) {
                std::string id = std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(itheta);
                double x, y;
                Index2Pose(ix, iy, grid_map_, &x, &y);
                double theta = itheta * phi_resolution_ + (-M_PI);
//                std::cout << "x: " << x << " y : " << y << " phi: " << theta << std::endl;
                bool result = CheckPose3d(x, y, theta, adc_length_, adc_width_, axle_ref_x_, 1.5, grid_map_);
                collision_lookup_table.emplace(id, result);
            }
        }
    }
//    for (const auto& item : collision_lookup_table){
//        std::cout << "index: " << item.first << ", result: " << item.second << std::endl;
//    }
    return collision_lookup_table;
}

std::unordered_map<std::string, double> HybridAStar::GenerateHeuristicCostLookUpTable() {
    std::unordered_map<std::string, double> heuristic_lookup_table;
    int phi_size = std::lround(2.0 * M_PI / phi_resolution_);
    for (int ix = 0; ix < grid_map_->info.width; ix++) {
        for (int iy = 0; iy < grid_map_->info.height; iy++) {
            for (int itheta = 0; itheta < phi_size; itheta++) {
                std::string id = std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(itheta);
                double x, y, theta;
                Index2Pose(ix, iy, grid_map_, &x, &y);
                theta = itheta * phi_resolution_ - M_PI;
                double holon_with_obstacle_h = heuristic_generator_->CheckDpMap(x, y);
                std::shared_ptr<Node3d> node = std::make_shared<Node3d>(x, y, theta, grid_map_, phi_resolution_);
                double nonholon_without_obstacle_h = NonHoloWithoutObstacleHeuristic(node);
                heuristic_lookup_table.emplace(id, std::max(holon_with_obstacle_h, nonholon_without_obstacle_h));
            }
        }
    }
    return heuristic_lookup_table;
}

void HybridAStar::RunOnce() {
    if (!has_grid_map_ || !valid_start_ || !valid_goal_) {
        ROS_FATAL("[HybridAStar::RunOnce: has_grid_map_ : %s, "
                  "valid_start_: %s, valid_goal_: %s]",
                  has_grid_map_ ? "true" : "false",
                  valid_start_ ? "true" : "false",
                  valid_goal_ ? "true" : "false");
        return;
    }

    std::cout << "start_pose: x: " << start_pose_.pose.pose.position.x << "y: "
              << start_pose_.pose.pose.position.y << "phi: "
              << tf::getYaw(start_pose_.pose.pose.orientation) << std::endl;
    int start_id_x, start_id_y;
    Pose2Index(start_pose_.pose.pose.position.x, start_pose_.pose.pose.position.y, grid_map_, &start_id_x, &start_id_y);
    double start_x, start_y;

    Index2Pose(start_id_x, start_id_y, grid_map_, &start_x, &start_y);
    if (std::fabs(start_x - start_pose_.pose.pose.position.x) < grid_map_->info.resolution
        && std::fabs(start_y - start_pose_.pose.pose.position.y) < grid_map_->info.resolution) {
        ROS_WARN("[The Index2Pose Is correct]");
    } else {
        ROS_WARN("[the Index2Pose Is Wrong]");
    }
    std::cout << "goal_pose: x: " << goal_pose_.pose.position.x <<
              " y : " << goal_pose_.pose.position.y << "phi: " <<
              tf::getYaw(goal_pose_.pose.orientation) << std::endl;
    HybridAStarResult result;
//    AStarResult result;
    double sx = start_pose_.pose.pose.position.x;
    double sy = start_pose_.pose.pose.position.y;
    double sphi = tf::getYaw(start_pose_.pose.pose.orientation);
    double ex = goal_pose_.pose.position.x;
    double ey = goal_pose_.pose.position.y;
    double ephi = tf::getYaw(goal_pose_.pose.orientation);
//    auto plan_result = heuristic_generator_->SearchPath(sx, sy, ex, ey, &result);
    ros::Time t0 = ros::Time::now();

    auto plan_result = this->Search(sx, sy, sphi, ex, ey, ephi, &result);
    ros::Time t1 = ros::Time::now();
    ROS_WARN(" SearchTime: %lf ms", static_cast<double>((t1 - t0).toNSec()) / 1000000.0);
    if (!plan_result) {
        ROS_FATAL("[HybridAStar::RunOnce]: HybridAStar Search Failed");
        return;
    }
    ROS_WARN("RunOnce, the plan_result: %s", plan_result ? "true" : "false");
    ROS_WARN("RunOnce, result size:  %zu", result.y.size());
    nav_msgs::Path published_path = this->HybridAStarResult2NavPath(&result);
//    nav_msgs::Path published_path = this->AStarResult2NavPath(result);
    ha_star_path_publisher_.publish(published_path);
    VisualizedPath(published_path);

}

void HybridAStar::VisualizedPath(const nav_msgs::Path &path) {
    visualization_msgs::Marker pathNode;
    visualization_msgs::MarkerArray pathNodes;
    pathNode.action = visualization_msgs::Marker::DELETE;
    visualization_msgs::Marker pathVehicle;
    visualization_msgs::MarkerArray pathVehicles;
    pathVehicle.header.frame_id = "/map";
    pathVehicle.header.stamp = ros::Time(0);

    pathVehicle.type = visualization_msgs::Marker::CUBE;
    pathVehicle.scale.x = adc_length_;
    pathVehicle.scale.y = adc_width_;
    pathVehicle.scale.z = 1;
    pathVehicle.color.a = 0.1;
    pathNode.action = visualization_msgs::Marker::ADD;
    pathNode.header.frame_id = "/map";
    pathNode.header.stamp = ros::Time::now();
    pathNode.type = visualization_msgs::Marker::SPHERE;
    pathNode.scale.x = 0.1;
    pathNode.scale.y = 0.1;
    pathNode.scale.z = 0.1;
    pathNode.color.a = 1.0;
    pathNode.color.r = 1.0;
    pathNode.color.g = 0.5;
    pathNode.color.b = 0.5;
    pathNodes.markers.clear();
    pathVehicles.markers.clear();
//    geometry_msgs::Point pt;
    for (size_t i = 0; i < path.poses.size(); ++i) {
        pathNode.id = i;
        pathVehicle.id = i;
        pathVehicle.pose = path.poses[i].pose;
        pathNode.pose.position.x = path.poses[i].pose.position.x;
        pathNode.pose.position.y = path.poses[i].pose.position.y;
        pathNode.pose.orientation = path.poses[i].pose.orientation;
        pathNodes.markers.push_back(pathNode);
        pathVehicles.markers.push_back(pathVehicle);
    }
    visualized_vehicle_publisher_.publish(pathVehicles);
    visulized_path_publisher_.publish(pathNodes);

}

nav_msgs::Path HybridAStar::AStarResult2NavPath(planning::AStarResult astar_result) {
    size_t result_size = astar_result.x.size();
    nav_msgs::Path path;
    path.header.frame_id = "/path";
    path.header.stamp = ros::Time::now();
    path.poses.resize(result_size);
    for (size_t i = 0; i < result_size; ++i) {
        const double x = (astar_result).x[i];
        const double y = (astar_result).y[i];
        path.poses[i].pose.position.x = x;
        path.poses[i].pose.position.y = y;
    }
    return path;
}

nav_msgs::Path HybridAStar::HybridAStarResult2NavPath(planning::HybridAStarResult *result) {
    size_t result_size = result->x.size();
    nav_msgs::Path path;
    path.header.frame_id = "/path";
    path.header.stamp = ros::Time::now();
    path.poses.resize(result_size);
    for (size_t i = 0; i < result_size; ++i) {
        const double x = (*result).x[i];
        const double y = (*result).y[i];
        const double phi = (*result).phi[i];
        path.poses[i].pose.position.x = x;
        path.poses[i].pose.position.y = y;
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, phi);
        geometry_msgs::Quaternion quaternion;

        tf::quaternionTFToMsg(q, quaternion);
        path.poses[i].pose.orientation = quaternion;
    }
    return path;
}

bool HybridAStar::Search(double sx,
                         double sy,
                         double sphi,
                         double ex,
                         double ey,
                         double ephi,
                         planning::HybridAStarResult *result) {
    if (result == nullptr) {
        ROS_ERROR("[HybridAStar::Search] : the input pointer is nullptr");
        return false;
    }
    result->x.clear();
    result->y.clear();
    result->phi.clear();
    result->steer.clear();
    result->a.clear();
    result->accumulated_s.clear();
    result->v.clear();
    start_node_.reset(
        new Node3d({sx}, {sy}, {sphi}, grid_map_, phi_resolution_));
    start_node_->SetDirection(true);
//    end_node_.reset(new Node3d(
//        {ex}, {ey}, {ephi}, grid_map_, phi_resolution_));
    open_set_.clear();
    open_pq_ = decltype(open_pq_)();
//    priority_queue_.clear();
//    open_set_handles_.clear();
    closed_set_.clear();
    final_node_ = nullptr;
    if (!CheckNode3d(start_node_)) {
        ROS_ERROR("[HybridAStar::Search], the start node is not collision free");
        return false;
    }
    ros::Time begin = ros::Time::now();
    if (!CheckNode3d(end_node_)) {
        ROS_ERROR("[HybridAStar::Search], the end node is not collision free");
        return false;
    }
    ros::Time end = ros::Time::now();
    ROS_WARN("[CheckNode3d] time elapsed: %lf ms", (end - begin).toNSec() / 1000000.0);
//    if (!heuristic_generator_->GenerateDpMap(ex, ey)) {
//        ROS_FATAL("[Search] , to generate dp map failed");
//        return false;
//    }
//    auto collision_lookup_table = GenerateCollisionCheckLookUpTable();
//    auto heuristic_lookup_table = GenerateHeuristicCostLookUpTable();
//    open_set_handles_.emplace(start_node_->GetIndex(), priority_queue_.emplace(start_node_));
    open_set_.emplace(start_node_->GetIndex(), start_node_);
    open_pq_.emplace(start_node_);
//    open_qp.emplace(start_node_->GetIndex(), start_node_->GetFCost());
    int expand_node_num = 0;
    double tie_breaker = 1.01;
    int cnt = 0;
    double analytic_expand_time = 0.0;
    double heuristic_calc_time = 0.0;
    double expand_node_time = 0.0;
    double check_node_time = 0.0;
    ros::Time tt0 = ros::Time::now();
    while (!open_pq_.empty()) {
        cnt++;
//        std::shared_ptr<Node3d> current_node = priority_queue_.top();

        const std::string current_id = open_pq_.top()->GetIndex();
        std::shared_ptr<Node3d> current_node = open_set_[current_id];
        open_pq_.pop();
        open_set_.erase(current_id);
//        open_set_handles_.erase(current_id);
        ros::Time t0 = ros::Time::now();
        if (current_node->IsInRange(end_node_, anlytic_expand_distance_)) {
            if (AnalyticExpansion(current_node)) {
                break;
            }
        }
        ros::Time t1 = ros::Time::now();
        analytic_expand_time += static_cast<double>((t1 - t0).toNSec()) / 1000000.0;
//        ROS_WARN("[AnalyticExpansion]: time: %lf ms", (end - begin).toNSec() / 1000000.0);
        if (*(current_node) == *(end_node_)) {
            final_node_ = current_node;
            break;
        }
        closed_set_.emplace(current_node->GetIndex(), current_node);
        t0 = ros::Time::now();
        for (size_t i = 0; i < next_node_num_; ++i) {
            std::shared_ptr<Node3d> next_node = std::move(NextNodeGenerator(current_node, i));
            if (next_node == nullptr) {
                continue;
            }
            if (closed_set_.find(next_node->GetIndex()) != closed_set_.end()) {
//                ROS_INFO("next node is in closed set");
                continue;
            }

//            bool check_result = CheckNode3d(next_node);
            bool check_result = true;
            int start_check_idx = 0;
            if (next_node->GetStepSize() == 1 ){
                start_check_idx = 0;
            } else{
                start_check_idx = 1;
            }
            for (int j = start_check_idx; j < next_node->GetStepSize(); ++j) {
                std::string id;
                double x = next_node->GetXs()[j];
                double y = next_node->GetYs()[j];
                double phi = next_node->GetPhis()[j];
                int ix, iy;
                Pose2Index(x, y, grid_map_, &ix, &iy);
                int iphi = static_cast<int>((phi + M_PI) / phi_resolution_);
                id = std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(iphi);
                if (!collision_lookup_table_[id]){
                    check_result = false;
                }
            }

            check_node_time += static_cast<double>((t1 - t0).toNSec()) / 1000000.0;
            if (!check_result) {
//                ROS_INFO("next node is not collision free, index : %zu", i);
                continue;
            }

            double tentative_gcost = current_node->GetGCost() +  0.35 * EdgeCost(current_node, next_node);
//            next_node->SetGCost(current_node->GetGCost() + EdgeCost(current_node, next_node));
            /// node not in open set, appead into openset and priority queue
            if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
                expand_node_num++;
                next_node->SetGCost( tentative_gcost);
//                next_node->SetHCost(HoloWithObstacleHeuristic(next_node));
//                t0 = ros::Time::now();
//                next_node->SetHCost(heuristic_lookup_table_[next_node->GetIndex()]);
                next_node->SetHCost(0.65 * std::max(
                    HoloWithObstacleHeuristic(next_node),
                    NonHoloWithoutObstacleHeuristic(next_node)));
//                std::cout << "g cost : " << next_node->GetGCost() << " hcost: " << next_node->GetFCost() << std::endl;
//                t1 = ros::Time::now();
//                heuristic_calc_time += static_cast<double>((t1 - t0).toNSec()) / 1000000.0;
                next_node->SetPreNode(current_node);
                open_set_.emplace(next_node->GetIndex(), next_node);
                open_pq_.emplace(next_node);
//                open_set_handles_.emplace(next_node->GetIndex(), next_handle);
            }
            t1 = ros::Time::now();
            expand_node_time += static_cast<double>((t1 - t0).toNSec()) / 1000000.0;
//            else {
//                if (open_set_[next_node->GetIndex()]->GetGCost() > tentative_gcost ) {
//                    open_set_[next_node->GetIndex()]->SetGCost(tentative_gcost);
//                    open_set_[next_node->GetIndex()]->SetPreNode(current_node);
////                    priority_queue_.push(open_set_[next_node->GetIndex()]);
////                    priority_queue_.update(open_set_handles_[next_node->GetIndex()],
////                                           open_set_[next_node->GetIndex()]);
//                }
//            }
            if (cnt > 100000) {
                return false;
            }

        }
    }
    ros::Time tt1 = ros::Time::now();
    ROS_WARN("expand total time: %lf ms", static_cast<double>((tt1 - tt0).toNSec())/ 1000000.0);
    ROS_WARN("analytic_expand_time: %lf, expand_node_time: %lf",
             analytic_expand_time, expand_node_time);
    ROS_WARN("[Search]: complete, the expanded node num is : %i", expand_node_num);
    if (final_node_ == nullptr) {
        ROS_ERROR("[Search], the final node_ is nullptr");
        return false;
    }
    return this->LoadResult(result);
}

bool HybridAStar::LoadResult(planning::HybridAStarResult *result) {
    if (result == nullptr) {
        return false;
    }
    std::shared_ptr<Node3d> current_node = final_node_;
    std::vector<double> hybrid_a_x;
    std::vector<double> hybrid_a_y;
    std::vector<double> hybrid_a_phi;
    while (current_node->GetPreNode() != nullptr) {
        std::vector<double> x = current_node->GetXs();
        std::vector<double> y = current_node->GetYs();
        std::vector<double> phi = current_node->GetPhis();
        if (x.empty() || y.empty() || phi.empty()) {
            return false;
        }
        if (x.size() != y.size() || x.size() != phi.size()) {
            return false;
        }
        std::reverse(x.begin(), x.end());
        std::reverse(y.begin(), y.end());
        std::reverse(phi.begin(), phi.end());
        x.pop_back();
        y.pop_back();
        phi.pop_back();
        hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
        hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
        hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
        current_node = current_node->GetPreNode();
    }
    hybrid_a_x.push_back(current_node->GetX());
    hybrid_a_y.push_back(current_node->GetY());
    hybrid_a_phi.push_back(current_node->GetPhi());
    std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
    std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
    std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
    (*result).x = hybrid_a_x;
    (*result).y = hybrid_a_y;
    (*result).phi = hybrid_a_phi;

    std::vector<HybridAStarResult> partitioned_results;
    if (!TrajectoryPartition(result, &partitioned_results)) {
        return false;
    }
    HybridAStarResult stitched_result;
    for (const auto &result : partitioned_results) {
        std::copy(result.x.begin(), result.x.end() - 1,
                  std::back_inserter(stitched_result.x));
        std::copy(result.y.begin(), result.y.end() - 1,
                  std::back_inserter(stitched_result.y));
        std::copy(result.phi.begin(), result.phi.end() - 1,
                  std::back_inserter(stitched_result.phi));
        std::copy(result.v.begin(), result.v.end() - 1,
                  std::back_inserter(stitched_result.v));
        std::copy(result.a.begin(), result.a.end(),
                  std::back_inserter(stitched_result.a));
        std::copy(result.steer.begin(), result.steer.end(),
                  std::back_inserter(stitched_result.steer));
    }
    stitched_result.x.push_back(partitioned_results.back().x.back());
    stitched_result.y.push_back(partitioned_results.back().y.back());
    stitched_result.phi.push_back(partitioned_results.back().phi.back());
    stitched_result.v.push_back(partitioned_results.back().v.back());
    if (stitched_result.x.size() != stitched_result.y.size() ||
        stitched_result.x.size() != stitched_result.v.size() ||
        stitched_result.x.size() != stitched_result.phi.size()) {
        return false;
    }
    if (stitched_result.a.size() != stitched_result.steer.size() ||
        stitched_result.x.size() - stitched_result.a.size() != 1) {
        return false;
    }
    *result = stitched_result;

    return true;
}

std::shared_ptr<Node3d> HybridAStar::NextNodeGenerator(
    std::shared_ptr<planning::Node3d> current_node,
    size_t next_node_index) {
    double steering = 0.0;
    double traveled_distance = 0.0;
    double step_size = std::sqrt(2) * grid_map_->info.resolution;
    if (next_node_index < static_cast<double>(next_node_num_) / 2) {
        steering =
            -max_steer_angle_ +
                (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
                    static_cast<double>(next_node_index);
        traveled_distance = step_size_;
    } else {
        size_t index = next_node_index - next_node_num_ / 2;
        steering =
            -max_steer_angle_ +
                (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
                    static_cast<double>(index);
        traveled_distance = -step_size_;
    }
    double arc = std::sqrt(2) * grid_map_->info.resolution;
    std::vector<double> intermediate_x;
    std::vector<double> intermediate_y;
    std::vector<double> intermediate_phi;
    double last_x = current_node->GetX();
    double last_y = current_node->GetY();
    double last_phi = current_node->GetPhi();
    intermediate_x.push_back(last_x);
    intermediate_y.push_back(last_y);
    intermediate_phi.push_back(last_phi);
    for (size_t i = 0; i < arc / step_size_; ++i) {
        const double next_x = last_x + traveled_distance * std::cos(last_phi);
        const double next_y = last_y + traveled_distance * std::sin(last_phi);
        const double next_phi = NormalizeAngle(
            last_phi +
                traveled_distance / wheel_base_ * std::tan(steering));
        intermediate_x.push_back(next_x);
        intermediate_y.push_back(next_y);
        intermediate_phi.push_back(next_phi);
        last_x = next_x;
        last_y = next_y;
        last_phi = next_phi;
    }
//    ROS_INFO("GenerateNextNode3d, next_node.GetStepSize() = %zu", intermediate_x.size());
    std::shared_ptr<Node3d> next_node = std::make_shared<Node3d>(
        intermediate_x, intermediate_y, intermediate_phi,
        grid_map_, phi_resolution_);

//    next_node->SetPreNode(current_node);
    next_node->SetSteering(steering);
    next_node->SetDirection(traveled_distance > 0.0);
//    next_node->SetGCost(current_node->GetGCost() + EdgeCost(current_node, next_node));
    return std::move(next_node);
}

double HybridAStar::EdgeCost(std::shared_ptr<planning::Node3d> current_node,
                             std::shared_ptr<planning::Node3d> next_node) {
    // backward motion
//    double edge_cost = 0.0;
    double edge_cost = (next_node->GetStepSize() - 1) * step_size_ * traj_step_length_penalty_;

//    double total_penalty = traj_forward_penalty_ + traj_backward_penalty_ + traj_gear_switch_penalty_
//        + traj_step_length_penalty_ + traj_gear_switch_penalty_ + traj_steer_penalty_ + traj_steer_change_penalty_;
//    ROS_INFO("[EdgeCost]: next_node->GetDirection() %s", next_node->GetDirection() ? "forward" : "backward");
    double coeff = 1.0;
    if (next_node->GetDirection()) {
        coeff *= traj_forward_penalty_;
//        edge_cost += traj_forward_penalty_ * (next_node->GetStepSize() - 1) * step_size_;
    } else {
        coeff *= traj_backward_penalty_;
//        edge_cost += traj_backward_penalty_ * (next_node->GetStepSize() - 1) * step_size_;
    }
//    edge_cost *= coeff;
    if (current_node->GetDirection() != next_node->GetDirection()) {
        coeff *= traj_gear_switch_penalty_;
//        ROS_INFO("[EdgeCost]: the current node direction diffs from next node direction");
//        edge_cost += traj_gear_switch_penalty_;
    }
//    double steer_cost = 0.0;
    if (std::fabs(next_node->GetSteering()) > phi_resolution_) {
        coeff *= traj_steer_penalty_;
//    steer_cost += traj_steer_penalty_ * std::fabs(next_node->GetSteering());
//    edge_cost += traj_steer_penalty_ * std::fabs(next_node->GetSteering());
    }
    if (std::fabs(next_node->GetSteering() - current_node->GetSteering()) > phi_resolution_) {
//    edge_cost += traj_steer_change_penalty_ *
//        std::fabs(next_node->GetSteering() - current_node->GetSteering());
        coeff *= traj_steer_change_penalty_;
    }

    return (edge_cost * coeff);

}

double HybridAStar::HoloWithObstacleHeuristic(
    std::shared_ptr<planning::Node3d> next_node) {
//    return std::hypot(next_node->GetX() - end_node_->GetX(), next_node->GetY() - end_node_->GetY());
    return (heuristic_generator_->CheckDpMap(
        next_node->GetX(),
        next_node->GetY()));
//    return h;
}

double HybridAStar::NonHoloWithoutObstacleHeuristic(
    std::shared_ptr<planning::Node3d> next_node) {

    ////////////////// dubin cost //////////////////////
    double non_holo_heu = 0.0;
    double dubin_cost = 0.0;
    double reedsShepp_cost = 0.0;
    const double min_r = 1.0 / (std::tan(max_steer_angle_) / wheel_base_);
    if (!enable_backward_) {
        ompl::base::DubinsStateSpace dubin_path(min_r);
        auto dubin_start = (ompl::base::SE2StateSpace::StateType *) dubin_path.allocState();
        auto dubin_end = (ompl::base::SE2StateSpace::StateType *) dubin_path.allocState();
        dubin_start->setXY(next_node->GetX(), next_node->GetY());
        dubin_start->setYaw(next_node->GetPhi());
        dubin_end->setXY(end_node_->GetX(), end_node_->GetY());
        dubin_end->setYaw(end_node_->GetPhi());
        dubin_cost = dubin_path.distance(dubin_start, dubin_end);
        non_holo_heu = dubin_cost;
    } else {
        /////////////////////////////reedsShepp cost ////////////
        ompl::base::ReedsSheppStateSpace reeds_shepp_path(min_r);
        auto rs_start = (ompl::base::ReedsSheppStateSpace::StateType *) reeds_shepp_path.allocState();
        auto rs_end = (ompl::base::ReedsSheppStateSpace::StateType *) reeds_shepp_path.allocState();
        rs_start->setXY(next_node->GetX(), next_node->GetY());
        rs_start->setYaw(next_node->GetPhi());
        rs_end->setXY(end_node_->GetX(), end_node_->GetY());
        rs_end->setYaw(end_node_->GetPhi());
        reedsShepp_cost = reeds_shepp_path.distance(rs_start, rs_end);
//    ROS_WARN("rs cost : %lf, dubin cost : %lf", reedsShepp_cost, dubin_cost);
        non_holo_heu = reedsShepp_cost;
    }

    return non_holo_heu;
}

bool HybridAStar::CheckNode3d(std::shared_ptr<Node3d> node) {
    size_t node_step_size = node->GetStepSize();
    auto traversed_x = node->GetXs();
    auto traversed_y = node->GetYs();
    auto traversed_phi = node->GetPhis();
    size_t check_start_index = 0;
    if (node_step_size == 1) {
        check_start_index = 0;
    } else {
        check_start_index = 1;
    }
//    std::cout << "node_step_size: " << node_step_size << std::endl;
    for (size_t i = check_start_index; i < node_step_size; ++i) {
        const double x = traversed_x[i];
        const double y = traversed_y[i];
        const double phi = traversed_phi[i];
        if (!CheckPose3d(x, y, phi, adc_length_,
                         adc_width_, axle_ref_x_, 1.5, grid_map_)) {
//            ROS_INFO("[CheckNode3d], the constraint check failed");
            return false;
        }
    }
    return true;
}

bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
    if (current_node == nullptr) {
        ROS_ERROR("[HybridAStar::AnalyticExpansion], "
                  "the input current node is nullptr");
        return false;
    }
    double dubin_length = 0.0;
    double reedsSheep_length = 0.0;
    if (enable_backward_) {
        auto final_node = this->ReedsSheppShot(
            current_node, &reedsSheep_length);
        if (final_node == nullptr) {
            return false;
        } else {
            final_node_ = std::move(final_node);
            final_node_->SetPreNode(current_node);
            closed_set_.emplace(final_node_->GetIndex(), final_node_);
            return true;
        }
    } else {
        auto final_node = this->DubinShot(current_node, &dubin_length);
        if (final_node == nullptr) {
            return false;
        } else {
            final_node_ = std::move(final_node);
            final_node_->SetPreNode(current_node);
            closed_set_.emplace(final_node_->GetIndex(), final_node_);
            return true;
        }
    }
}

std::shared_ptr<Node3d> HybridAStar::DubinShot(std::shared_ptr<planning::Node3d> node, double *len) {
    if (node == nullptr) {
        return nullptr;
    }
    const double min_r = 1.0 / (std::tan(max_steer_angle_) / wheel_base_);
    ompl::base::StateSpacePtr space(std::make_shared<ompl::base::DubinsStateSpace>(min_r));
    ompl::base::ScopedState<> from(space), to(space), s(space);
    from[0] = node->GetX();
    from[1] = node->GetY();
    from[2] = node->GetPhi();
    to[0] = end_node_->GetX();
    to[1] = end_node_->GetY();
    to[2] = end_node_->GetPhi();
    std::vector<double> reals;
    *len = space->distance(from(), to());
    double ratio = step_size_ / *len;
    double t = 0.0;
    std::vector<double> traversed_x, traversed_y, traversed_phi;
    while (t < 1.0 + ratio) {
        space->interpolate(from(), to(), t, s());
        reals = s.reals();
//        std::cout << "x: " << reals[0] << ", y: " << reals[1] << ", z: " << reals[2] << std::endl;
        traversed_x.push_back(reals[0]);
        traversed_y.push_back(reals[1]);
        traversed_phi.push_back(reals[2]);
        t += ratio;
    }
    auto final_node =
        std::make_shared<Node3d>(
            traversed_x,
            traversed_y,
            traversed_phi,
            grid_map_,
            phi_resolution_);
    if (!CheckNode3d(final_node)) {
        return nullptr;
    }
    return final_node;
}

std::shared_ptr<Node3d> HybridAStar::ReedsSheppShot(
    std::shared_ptr<planning::Node3d> node, double *len) {

    if (node == nullptr) {
        return nullptr;
    }
    const double min_r = 1.0 / (std::tan(max_steer_angle_) / wheel_base_);
    ompl::base::StateSpacePtr space(std::make_shared<ompl::base::ReedsSheppStateSpace>(min_r));
    ompl::base::ScopedState<> from(space), to(space), s(space);
    from[0] = node->GetX();
    from[1] = node->GetY();
    from[2] = node->GetPhi();
    to[0] = end_node_->GetX();
    to[1] = end_node_->GetY();
    to[2] = end_node_->GetPhi();
    std::vector<double> reals;
    *len = space->distance(from(), to());
    double delta_t = 0.1;
    double ratio = step_size_ / *len;
    double t = 0.0;
    std::vector<double> traversed_x, traversed_y, traversed_phi;
    while (t < 1.0 + ratio) {
        space->interpolate(from(), to(), t, s());
        reals = s.reals();
//        std::cout << "x: " << reals[0] << ", y: " << reals[1] << ", z: " << reals[2] << std::endl;

        traversed_x.push_back(reals[0]);
        traversed_y.push_back(reals[1]);
        traversed_phi.push_back(reals[2]);
        t += ratio;
    }
    auto final_node =
        std::make_shared<Node3d>(
            traversed_x,
            traversed_y,
            traversed_phi,
            grid_map_,
            phi_resolution_);
    if (!CheckNode3d(final_node)) {
        return nullptr;
    }
    return final_node;
}

bool HybridAStar::TrajectoryPartition(
    const planning::HybridAStarResult *result,
    std::vector<planning::HybridAStarResult> *parition_results) {
    if (result == nullptr || parition_results == nullptr) {
        return false;
    }
    const auto &x = result->x;
    const auto &y = result->y;
    const auto &phi = result->phi;
    if (x.size() != y.size() || x.size() != phi.size()) {
        return false;
    }
    size_t horizon = x.size();
    parition_results->clear();
    parition_results->emplace_back();
    auto *current_traj = &(parition_results->back());
    double heading_angle = phi.front();
    double tracking_angle = std::atan2(y[1] - y[0], x[1] - x[0]);
    bool current_gear = std::fabs(CalcAngleDist(tracking_angle, heading_angle)) < M_PI_2;
    for (size_t i = 0; i < horizon - 1; ++i) {
        heading_angle = phi[i];
        tracking_angle = std::atan2(y[i + 1] - y[i], x[i + 1] - x[i]);
        bool gear = std::fabs(CalcAngleDist(tracking_angle, heading_angle)) < M_PI_2;
        if (gear != current_gear) {
            current_traj->x.push_back(x[i]);
            current_traj->y.push_back(y[i]);
            current_traj->phi.push_back(phi[i]);
            parition_results->emplace_back();
            current_traj = &(parition_results->back());
            current_gear = gear;
        }
        current_traj->x.push_back(x[i]);
        current_traj->y.push_back(y[i]);
        current_traj->phi.push_back(phi[i]);
    }
    current_traj->x.push_back(x.back());
    current_traj->y.push_back(y.back());
    current_traj->phi.push_back(phi.back());
    for (auto &traj : *parition_results) {
        if (traj.x.size() < 2 || traj.y.size() < 2 || traj.phi.size() < 2) {
            return false;
        }
        size_t x_size = traj.x.size();
        traj.v.push_back(0.0);

        for (size_t i = 1; i + 1 < x_size; ++i) {
            double discrete_v = (((traj.x[i + 1] - traj.x[i]) / delta_t_) *
                std::cos(traj.phi[i]) +
                ((traj.x[i] - traj.x[i - 1]) / delta_t_) *
                    std::cos(traj.phi[i])) /
                2.0 +
                (((traj.y[i + 1] - traj.y[i]) / delta_t_) *
                    std::sin(traj.phi[i]) +
                    ((traj.y[i] - traj.y[i - 1]) / delta_t_) *
                        std::sin(traj.phi[i])) /
                    2.0;
            traj.v.push_back(discrete_v);
        }
        traj.v.push_back(0.0);
        for (size_t i = 0; i < x_size - 1; ++i) {
            const double discrete_a = (traj.v[i + 1] - traj.v[i]) / delta_t_;
            traj.a.push_back(discrete_a);
        }
        for (size_t i = 0; i + 1 < x_size; ++i) {
            double discrete_steer = (traj.phi[i + 1] - traj.phi[i]) *
                wheel_base_ / step_size_;
            if (traj.v[i] > 0.0) {
                discrete_steer = std::atan(discrete_steer);
            } else {
                discrete_steer = std::atan(-discrete_steer);
            }
            traj.steer.push_back(discrete_steer);
        }

    }
    return true;
}
}