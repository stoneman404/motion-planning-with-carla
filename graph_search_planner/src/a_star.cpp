#include "a_star.hpp"
#include "graph_search_utils.hpp"
#include <queue>
#include <boost/heap/binomial_heap.hpp>
namespace planning {

std::vector<std::shared_ptr<Node2d>> AStar::GenerateNextNodes(
    std::shared_ptr<planning::Node2d> node) {
    std::vector<std::shared_ptr<Node2d>> next_nodes;
    int cur_index_x = node->GetIndexX();
    int cur_index_y = node->GetIndexY();
    double current_x, current_y;
    Index2Pose(cur_index_x, cur_index_y, grid_map_, &current_x, &current_y);
    double xy_resolution = grid_map_->info.resolution;
    std::shared_ptr<Node2d> up = std::make_shared<Node2d>(cur_index_x, cur_index_y + 1);
//    up->SetGcost(node->GetGCost() + xy_resolution);
    std::shared_ptr<Node2d> down = std::make_shared<Node2d>(cur_index_x, cur_index_y - 1);
    down->SetGcost(node->GetGCost() + xy_resolution);
    std::shared_ptr<Node2d> right = std::make_shared<Node2d>(cur_index_x + 1, cur_index_y);
//    right->SetGcost(node->GetGCost() + xy_resolution);
    auto left = std::make_shared<Node2d>(cur_index_x - 1, cur_index_y);
//    left->SetGcost(node->GetGCost() + xy_resolution);
    auto up_right = std::make_shared<Node2d>(cur_index_x + 1, cur_index_y + 1);
//    up_right->SetGcost(node->GetGCost() + std::sqrt(2.0) * xy_resolution);
    auto up_left = std::make_shared<Node2d>(cur_index_x - 1, cur_index_y + 1);
//    up_left->SetGcost(node->GetGCost() + std::sqrt(2.0) * xy_resolution);
    auto down_right = std::make_shared<Node2d>(cur_index_x + 1, cur_index_y - 1);
//    up_left->SetGcost(node->GetGCost() + std::sqrt(2.0) * xy_resolution);
    auto down_left = std::make_shared<Node2d>(cur_index_x - 1, cur_index_y - 1);
//    down_left->SetGcost(node->GetGCost() + std::sqrt(2.0) * xy_resolution);
    next_nodes.emplace_back(up);
    next_nodes.emplace_back(down);
    next_nodes.emplace_back(right);
    next_nodes.emplace_back(left);
    next_nodes.emplace_back(up_right);
    next_nodes.emplace_back(up_left);
    next_nodes.emplace_back(down_right);
    next_nodes.emplace_back(down_left);
    return next_nodes;
}

// we treat the robot in a star algorithm as a point robot,
// because it only provides heuristic for hybrid a star
bool AStar::VerifyNode2d(std::shared_ptr<planning::Node2d> node) {
    int index_x = node->GetIndexX();
    int index_y = node->GetIndexY();
    double x, y;

//    double raidus = std::hypot(0.5 * length_ + axle_ref_x_, 0.5 * width_);
//    if (length_ > width_){
//        raidus = axle_ref_x_ + 0.5 * length_ + grid_map_->info.resolution;
//    }else{
//        raidus = 0.5 * width_ + grid_map_->info.resolution;
//    }
    Index2Pose(index_x, index_y, grid_map_, &x, &y);
    return CheckPose2d(x, y, grid_map_);
//    return CheckCircleRobotPose(x, y, raidus, grid_map_);
}

bool AStar::TracePath(AStarResult *astar_path) {
    if (astar_path == nullptr) {
        ROS_FATAL("[AStar::TracePath]: the input pointer is nullptr");
        return false;
    }
    if (final_node_ == nullptr) {
        ROS_FATAL("[AStar::TracePath]: the final node is nullptr, cannot find path");
        return false;
    }
    std::vector<double> xs, ys;
    auto current_node = final_node_;
    double x, y;
    while (current_node->GetPreNode() != nullptr) {

        Index2Pose(
            current_node->GetIndexX(),
            current_node->GetIndexY(),
            grid_map_,
            &x, &y);
        std::cout << " index_x : " << current_node->GetIndexX()
                  << " index_y: " << current_node->GetIndexY()
                  << " x: " << x << " y : " << y << std::endl;

        xs.push_back(x);
        ys.push_back(y);
        current_node = current_node->GetPreNode();
    }
    double x_start, y_start;
    Index2Pose(
        current_node->GetIndexX(),
        current_node->GetIndexY(),
        grid_map_,
        &x_start,
        &y_start);
    std::cout << " index_x : " << current_node->GetIndexX()
              << " index_y: " << current_node->GetIndexY()
              << " x: " << x_start << " y : " << y_start << std::endl;
    xs.push_back(x_start);
    ys.push_back(y_start);
    std::reverse(xs.begin(), xs.end());
    std::reverse(ys.begin(), ys.end());
    astar_path->x = std::move(xs);
    astar_path->y = std::move(ys);
    std::cout << "=============== a star result ===============" << std::endl;
    double len = 0.0;
    for (size_t i = 1; i < astar_path->x.size(); ++i) {
        len += std::hypot(astar_path->x[i] - astar_path->x[i - 1], astar_path->y[i] - astar_path->y[i - 1]);
        std::cout << "x: " << astar_path->x[i] << " y: " << astar_path->y[i] << std::endl;
    }
    astar_path->cost = len;
    return true;
}

//bool AStar::SearchPath(double sx, double sy, double ex, double ey, AStarResult *result) {
//    if (result == nullptr) {
//        ROS_FATAL("[AStar::SearchPath], the input pointer is nullptr");
//        return false;
//    }
//    result->x.clear();
//    result->y.clear();
////    start_node_ = std::make_shared<Node2d>(sx, sy, grid_map_);
////    goal_node_ = std::make_shared<Node2d>(ex, ey, grid_map_);
//
//
////    std::priority_queue<
////        std::pair<std::string, double>,
////        std::vector<std::pair<std::string, double>>, cmp
////    > pq;
////    std::unordered_map<std::string, std::shared_ptr<Node2d>> closed_set;
////    std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
////    final_node_ = nullptr;
////    start_node_->SetPreNode(nullptr);
////    start_node_->SetGcost(0.0);
////    start_node_->SetHCost(EuclidDistance(start_node_, goal_node_));
////    goal_node_->SetGcost(std::numeric_limits<double>::max());
////    goal_node_->SetHCost(0.0);
////    goal_node_->SetPreNode(nullptr);
////    open_set.emplace(start_node_->GetIndex(), start_node_);
////    pq.emplace(start_node_->GetIndex(), start_node_->GetFCost());
////    while (!pq.empty()) {
////        std::string current_id = pq.top().first;
////        pq.pop();
////        std::shared_ptr<Node2d> current_node = open_set[current_id];
////        if (*(current_node) == *(goal_node_)) {
////            final_node_ = current_node;
////            break;
////        }
////        closed_set.emplace(current_node->GetIndex(), current_node);
////        std::vector<std::shared_ptr<Node2d>> next_nodes = std::move(
////            this->GenerateNextNodes(current_node));
////        for (auto &next_node : next_nodes) {
////
////            if (!this->VerifyNode2d(next_node)) {
////                continue;
////            }
////            // if the node is in closed set
////            if (closed_set.find(next_node->GetIndex()) != closed_set.end()) {
////                continue;
////            }
////            // if the node is not in open set and closed set, a new node
////            if (open_set.find(next_node->GetIndex()) == open_set.end()) {
////                next_node->SetHCost(this->EuclidDistance(next_node, goal_node_));
////                next_node->SetPreNode(current_node);
////                open_set.emplace(next_node->GetIndex(), next_node);
////                pq.emplace(next_node->GetIndex(), next_node->GetFCost());
////
////            } else if (open_set[next_node->GetIndex()]->GetGCost() > next_node->GetGCost()) {
////                // in open set but we have a new g cost less than origin
////                open_set[next_node->GetIndex()]->SetGcost(next_node->GetGCost());
////                open_set[next_node->GetIndex()]->SetPreNode(current_node);
////                pq.emplace(next_node->GetIndex(), next_node->GetFCost());
////            }
////        }
////    }
//    boost::heap::binomial_heap<std::shared_ptr<Node2d>, boost::heap::compare<cmp>> priority_queue;
//    std::unordered_map<std::string, boost::heap::binomial_heap<std::shared_ptr<Node2d>,
//                                                               boost::heap::compare<cmp>>::handle_type>
//        open_set_handles;
//    std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
//    std::unordered_map<std::string, std::shared_ptr<Node2d>> close_set;
//    std::shared_ptr<Node2d> start_node =
//        std::make_shared<Node2d>(sx, sy, grid_map_);
//    std::shared_ptr<Node2d> end_node =
//        std::make_shared<Node2d>(ex, ey, grid_map_);
//
//    if (!VerifyNode2d(start_node)) {
//        ROS_FATAL("[AStar::SearchPath], the start node is not collision free");
//        return false;
//    }
//    if (!VerifyNode2d(end_node)) {
//        ROS_FATAL("[AStar::SearchPath], the end node is not collision free");
//        return false;
//    }
//    final_node_ = nullptr;
//    open_set.emplace(start_node->GetIndex(), start_node);
//
//    auto handle = priority_queue.emplace(start_node);
//    open_set_handles.emplace(start_node->GetIndex(), handle);
//    size_t explored_node_num = 0;
//    while (!priority_queue.empty()) {
//        std::string current_id = priority_queue.top()->GetIndex();
//        priority_queue.pop();
//        std::shared_ptr<Node2d> current_node = open_set[current_id];
//        open_set_handles.erase(current_id);
//        open_set.erase(current_id);
//
//        if (*(current_node) == *(end_node)) {
//            final_node_ = current_node;
//            break;
//        }
//        close_set.emplace(current_node->GetIndex(), current_node);
//        std::vector<std::shared_ptr<Node2d>> next_nodes =
//            std::move(GenerateNextNodes(current_node));
//        for (auto &next_node : next_nodes) {
//            if (!VerifyNode2d(next_node)) {
//                continue;
//            }
//            if (close_set.find(next_node->GetIndex()) != close_set.end()) {
//                continue;
//            }
//            if (open_set.find(next_node->GetIndex()) == open_set.end()) {
//                ++explored_node_num;
//                next_node->SetHCost(2.0 *
//                    MahDistance(next_node, end_node));
//                next_node->SetPreNode(current_node);
//                next_node->SetGcost(current_node->GetGCost() + EdgeCost(current_node, next_node));
//                open_set.emplace(next_node->GetIndex(), next_node);
//                auto next_handle = priority_queue.emplace(next_node);
//                open_set_handles.emplace(next_node->GetIndex(), next_handle);
//            } else if (open_set[next_node->GetIndex()]->GetGCost() < next_node->GetGCost()) {
//                open_set[next_node->GetIndex()] = next_node;
//                open_set[next_node->GetIndex()]->SetGcost(next_node->GetGCost());
//                open_set[next_node->GetIndex()]->SetPreNode(current_node);
////                priority_queue.update(open_set_handles[next_node->GetIndex()], next_node);
//            }
//        }
//    }
//
//    if (final_node_ == nullptr) {
//        ROS_FATAL("[AStar::SearchPath]:, the final node is nullptr, Failed to find Path");
//        return false;
//    }
//    this->TracePath(result);
//    return true;
//}

bool AStar::GenerateDpMap(double ex, double ey) {
    auto end_node = std::make_shared<Node2d>(ex, ey, grid_map_);
    if (!VerifyNode2d(end_node)) {
        ROS_FATAL("[AStar::GenerateDpMap], the end node is not collision free");
        return false;
    }
    dp_map_.clear();
    std::priority_queue<std::shared_ptr<Node2d>, std::vector<std::shared_ptr<Node2d>>, cmp> open_pq;
//    boost::heap::binomial_heap<
//        std::shared_ptr<Node2d>,
//        boost::heap::compare<cmp>> priority_queue;
//    std::unordered_map<std::string,
//                       boost::heap::binomial_heap<
//                           std::shared_ptr<Node2d>,
//                           boost::heap::compare<cmp>>::handle_type> open_set_handles;
    std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
    end_node->SetPreNode(nullptr);
    end_node->SetFcost(0.0);
//    end_node->SetFcost(0.0);
    open_pq.emplace(end_node);
    open_set.emplace(end_node->GetIndex(), end_node);
//    auto handle = priority_queue.push(end_node);
//    open_set_handles.emplace(end_node->GetIndex(), handle);
    while (!open_pq.empty()) {
//        auto current_node = priority_queue.top();
//        std::string current_id = priority_queue.top()->GetIndex();
//        std::string current_id = current_node->GetIndex();
          std::string current_id = open_pq.top()->GetIndex();
//        ROS_WARN("GenerateDpMap: current_node : index: %s, Fcost: %lf",
//            current_node->GetIndex().c_str(),
//            current_node->GetFCost());
        auto current_node = open_set[current_id];
        open_pq.pop();
        open_set.erase(current_id);
//        open_set_handles.erase(current_id);
        dp_map_.emplace(current_node->GetIndex(), current_node);
        std::vector<std::shared_ptr<Node2d>> next_nodes =
            std::move(this->GenerateNextNodes(current_node));
        for (auto &next_node : next_nodes) {
            if (!VerifyNode2d(next_node)) {
                continue;
            }
            // dp_map_ acts as closed set
            if (dp_map_.find(next_node->GetIndex()) != dp_map_.end()) {
                continue;
            }
            double tentative_f = current_node->GetFCost() + EdgeCost(current_node, next_node);
            if (open_set.find(next_node->GetIndex()) == open_set.end()) {
                next_node->SetPreNode(current_node);
                next_node->SetFcost(tentative_f);
//                ROS_WARN("next_node, %s, FCost: %lf, EdgeCost: %lf",
//                         next_node->GetIndex().c_str(),
//                         next_node->GetFCost(),
//                         EdgeCost(current_node, next_node));
                open_set.emplace(next_node->GetIndex(), next_node);
                open_pq.emplace(next_node);
//                auto next_handle = priority_queue.push(next_node);
//                open_set_handles.emplace(next_node->GetIndex(), next_handle);
//                ROS_WARN("[AfterPush], open_set[%s]: FCost: %lf", next_node->GetIndex().c_str(),
//                    open_set[next_node->GetIndex()]->GetFCost());
            }
            else if (open_set[next_node->GetIndex()]->GetFCost() > tentative_f) {
                open_set[next_node->GetIndex()]->SetFcost(tentative_f);
                open_set[next_node->GetIndex()]->SetPreNode(current_node);
//                priority_queue.update(open_set_handles[next_node->GetIndex()],
//                    open_set[next_node->GetIndex()]);
            }
        }
    }
//    ROS_INFO("end_node index =%s", end_node->GetIndex().c_str());
////
//    for (auto &item :  dp_map_) {
//        std::cout << "index: " << item.first << ", fcost: " << item.second->GetFCost() << std::endl;
//    }

    return true;
}

double AStar::CheckDpMap(double sx, double sy) {
    int index_x, index_y;
    Pose2Index(sx, sy, grid_map_, &index_x, &index_y);
    std::string index = std::to_string(index_x) + "_" + std::to_string(index_y);
    if (dp_map_.find(index) != dp_map_.end()) {
        return dp_map_[index]->GetFCost();
    } else {
        return std::numeric_limits<double>::infinity();
    }
}
}