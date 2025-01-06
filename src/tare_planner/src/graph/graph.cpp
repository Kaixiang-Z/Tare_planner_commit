/**
 * @file graph.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a graph
 * @version 0.1
 * @date 2021-07-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <queue>
#include <ros/ros.h>
#include "utils/misc_utils.h"
#include "graph/graph.h"

namespace tare {
Graph::Graph(int node_number) {
    connection_.resize(node_number);
    distance_.resize(node_number);
    positions_.resize(node_number);
}

void Graph::AddNode(const Eigen::Vector3d &position) {
    // graph 增加一个节点， 推入一个空的connection， 邻接节点的距离，并传入自身位置
    std::vector<int> connection;
    connection_.push_back(connection);
    std::vector<double> neighbor_distance;
    distance_.push_back(neighbor_distance);
    positions_.push_back(position);
}

void Graph::SetNodePosition(int node_index, const Eigen::Vector3d &position) {
    // 传入节点的索引， 更新该索引对应节点的位置信息
    // 如果该node index已经存在，直接更新位置，若node index是最后一个 + 1，新增一个节点，否则报错
    if (NodeIndexInRange(node_index)) {
        positions_[node_index] = position;
    } else if (node_index == positions_.size()) {
        AddNode(position);
    } else {
        ROS_ERROR_STREAM("Graph::SetNodePosition: node_index: " << node_index << " not in range [0, "
                                                                << positions_.size() - 1 << "]");
    }
}

void Graph::AddOneWayEdge(int from_node_index, int to_node_index, double distance) {
    // 新增两节点的边的距离
    // 如果 from 和 to 节点都存在，更新connection以及distance信息，否则报错
    if (NodeIndexInRange(from_node_index) && NodeIndexInRange(to_node_index)) {
        connection_[from_node_index].push_back(to_node_index);
        distance_[from_node_index].push_back(distance);
    } else {
        ROS_ERROR_STREAM("Graph::AddOneWayEdge: from_node_index: " << from_node_index << " to_node_index: "
                                                                   << to_node_index << " not in range [0, "
                                                                   << connection_.size() - 1 << "]");
    }
}

void Graph::AddTwoWayEdge(int from_node_index, int to_node_index, double distance) {
    // 建立双向的边的信息
    AddOneWayEdge(from_node_index, to_node_index, distance);
    AddOneWayEdge(to_node_index, from_node_index, distance);
}

double Graph::GetShortestPath(int from_node_index, int to_node_index, bool get_path, nav_msgs::Path &shortest_path,
                              std::vector<int> &node_indices) {
    // 获取两节点间的最短路径
    // 使用A*算法， 返回path的长度
    node_indices.clear();
    double path_length = AStarSearch(from_node_index, to_node_index, get_path, node_indices);
    if (get_path) {
        shortest_path.poses.clear();
        for (const auto &node_index : node_indices) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = positions_[node_index].x();
            pose.pose.position.y = positions_[node_index].y();
            pose.pose.position.z = positions_[node_index].z();
            shortest_path.poses.push_back(pose);
        }
    }
    return path_length;
}

double Graph::AStarSearch(int from_node_index, int to_node_index, bool get_path, std::vector<int> &node_indices) {
    // 图的 A* 算法，获取两节点间的最短路径
    // 确保两个节点均在范围内
    MY_ASSERT(NodeIndexInRange(from_node_index));
    MY_ASSERT(NodeIndexInRange(to_node_index));

    double INF = 9999.0;
    typedef std::pair<double, int> iPair;
    double shortest_dist = 0;
    // 优先队列，最小堆，队首弹出最小代价的元素
    std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
    // 实际代价
    std::vector<double> g(connection_.size(), INF);
    // 估计代价
    std::vector<double> f(connection_.size(), INF);
    // 节点的前继节点
    std::vector<int> prev(connection_.size(), -1);
    // 判断该节点是否在邻接表中
    std::vector<bool> in_pg(connection_.size(), false);

    // 初始化
    g[from_node_index] = 0;
    f[from_node_index] = (positions_[from_node_index] - positions_[to_node_index]).norm();

    pq.push(std::make_pair(f[from_node_index], from_node_index));
    in_pg[from_node_index] = true;

    while (!pq.empty()) {
        // 弹出最小估计代价的节点索引
        int u = pq.top().second;
        pq.pop();
        // 表示已不在邻接表中
        in_pg[u] = false;
        // 如果到达终点，保存最短距离并退出
        if (u == to_node_index) {
            shortest_dist = g[u];
            break;
        }
        // 遍历该节点的所有connection
        for (int i = 0; i < connection_[u].size(); i++) {
            // 获取具有关联的节点索引
            int v = connection_[u][i];
            MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(connection_, v));
            // 获取该节点到当前节点的距离
            double d = distance_[u][i];
            // 松弛化操作，若该节点的实际代价大于当前节点的代价+距离，则更新信息
            if (g[v] > g[u] + d) {
                // 更新前继， 作为路径的一部分
                prev[v] = u;
                // 更新实际代价
                g[v] = g[u] + d;
                // 估计代价： 实际代价 + 该节点到目的节点的距离
                f[v] = g[v] + (positions_[v] - positions_[to_node_index]).norm();
                // 把该节点放回邻接表中
                if (!in_pg[v]) {
                    pq.push(std::make_pair(f[v], v));
                    in_pg[v] = true;
                }
            }
        }
    }
    // 如果需要得到路径，则根据前继表搜索，最后进行一个反序操作
    if (get_path) {
        node_indices.clear();
        int u = to_node_index;
        if (prev[u] != -1 || u == from_node_index) {
            while (u != -1) {
                node_indices.push_back(u);
                u = prev[u];
            }
        }
    }
    std::reverse(node_indices.begin(), node_indices.end());

    return shortest_dist;
}

} // namespace tare
