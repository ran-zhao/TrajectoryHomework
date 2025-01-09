#include <ros/ros.h>
#include <utility>
#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>
#include "nav_msgs/Path.h"
#include <memory>
#include <iostream>
#include <limits>

// 定义 Node 结构体
struct Node {
    int x, y;        // 节点所在的网格坐标
    double g_cost;   // 从起点到当前节点的代价
    double h_cost;   // 从当前节点到终点的估计代价
    std::shared_ptr<Node> parent;    // 父节点，用于回溯路径

    Node(int x, int y, double g_cost, double h_cost, std::shared_ptr<Node> parent = nullptr)
            : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(std::move(parent)) {}

    double f() const { return g_cost + h_cost; } // 总代价值
};

// 比较器，用于优先队列
struct cmp{
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b){
        return a->f() > b->f();
    }
};

// 定义 GridMap 结构体
struct GridMap {
    int width;
    int height;
    double map_max;
    double map_min;
    double grid_resolution;
    std::vector<std::vector<int>> grid; // 0: 空闲, 1: 占用

    GridMap(int w, int h, double map_min_, double map_max_, double res) 
        : width(w), height(h), map_min(map_min_), map_max(map_max_), grid_resolution(res), grid(w, std::vector<int>(h, 0)) {}

    void markObstacle(double cx, double cy, double radius) {
        int grid_cx = std::round((cx - map_min) / grid_resolution);
        int grid_cy = std::round((cy - map_min) / grid_resolution);
        int grid_radius = std::round(radius / grid_resolution);
        
        // 遍历以 (grid_cx, grid_cy) 为中心，半径为 grid_radius 的正方形区域
        for(int dx = -grid_radius; dx <= grid_radius; ++dx){
            for(int dy = -grid_radius; dy <= grid_radius; ++dy){
                int x = grid_cx + dx;
                int y = grid_cy + dy;
                // 检查坐标是否在地图范围内
                if(x >= 0 && x < width && y >= 0 && y < height){
                    // 计算网格单元中心到圆心的距离
                    double distance = std::sqrt(dx*dx + dy*dy) * grid_resolution;
                    if(distance <= radius){
                        grid[x][y] = 1; // 标记为占用
                    }
                }
            }
        }
    }
};

// 定义 AStarPlanner 类
class AStarPlanner {
public:
    AStarPlanner(int width, int height, double m_min, double m_max, double res) 
        : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res), 
          grid_map_(width, height, map_min_, map_max_, grid_resolution_), num_of_obs_(0) 
    {}

    void setObstacle(double cx, double cy, double radius) {
        num_of_obs_++;
        grid_map_.markObstacle(cx, cy, radius);
    }

    void printGridMap(){
        for(int i = 0; i < width_; i++){
            for(int j = 0; j < height_; j++){
                std::cout<<grid_map_.grid[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<"num of obstacles: "<<num_of_obs_<<std::endl;
    }

    std::vector<Eigen::Vector2d> findPath(Eigen::Vector2d start, Eigen::Vector2d goal) {
        if(num_of_obs_ == 0){
            return {};
        }
        // 起点和终点转换为网格坐标
        auto worldToGridSafe = [&](const Eigen::Vector2d& position) -> std::pair<int, int> {
            int x = std::round((position.x() - map_min_) / grid_resolution_);
            int y = std::round((position.y() - map_min_) / grid_resolution_);
            x = std::max(0, std::min(x, width_ - 1));
            y = std::max(0, std::min(y, height_ - 1));
            return {x, y};
        };

        auto gridStart = worldToGridSafe(start);
        auto gridGoal = worldToGridSafe(goal);

        // 检查起点和终点是否被占用
        if(grid_map_.grid[gridStart.first][gridStart.second] == 1 || 
           grid_map_.grid[gridGoal.first][gridGoal.second] == 1){
            ROS_WARN("Start or Goal is occupied!");
            return {};
        }

        // 开放列表和关闭列表
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list;
        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false));

        // 起点加入开放列表
        open_list.push(std::make_shared<Node>(Node(gridStart.first, gridStart.second, 0.0, heuristic(gridStart, gridGoal))));

        // 用于记录每个节点的最小 g_cost
        std::vector<std::vector<double>> g_costs(width_, std::vector<double>(height_, std::numeric_limits<double>::infinity()));
        g_costs[gridStart.first][gridStart.second] = 0.0;

        while(!open_list.empty()){
            auto current = open_list.top();
            open_list.pop();

            // 如果当前节点已经在关闭列表中，跳过
            if(closed_list[current->x][current->y]){
                continue;
            }

            // 将当前节点加入关闭列表
            closed_list[current->x][current->y] = true;

            // 如果到达目标，回溯路径
            if(current->x == gridGoal.first && current->y == gridGoal.second){
                return reconstructPath(current);
            }

            // 获取所有邻居
            std::vector<Node> neighbors = getNeighbors(*current);

            for(auto& neighbor : neighbors){
                if(closed_list[neighbor.x][neighbor.y]){
                    continue; // 已在关闭列表中
                }

                double tentative_g = current->g_cost + distance(*current, neighbor);

                // 如果找到更好的路径
                if(tentative_g < g_costs[neighbor.x][neighbor.y]){
                    g_costs[neighbor.x][neighbor.y] = tentative_g;
                    double h = heuristic({neighbor.x, neighbor.y}, gridGoal);
                    auto neighbor_ptr = std::make_shared<Node>(neighbor.x, neighbor.y, tentative_g, h, current);
                    open_list.push(neighbor_ptr);
                }
            }
        }

        // 如果没有找到路径，返回空路径
        ROS_WARN("No path found!");
        return {};
    }

    void reset(){
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }

private:
    // 计算启发式代价（使用欧几里得距离）
    double heuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2));
    }

    // 计算两节点之间的距离（用于邻居代价计算）
    double distance(const Node& a, const Node& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    // 从世界坐标转换到栅格坐标
    std::pair<int, int> worldToGrid(const Eigen::Vector2d& position) {
        int x = std::round((position.x() - map_min_) / grid_resolution_);
        int y = std::round((position.y() - map_min_) / grid_resolution_);
        return {x, y};
    }

    // 从栅格坐标转换到世界坐标（主要用于路径结果显示）
    Eigen::Vector2d gridToWorld(int x, int y) {
        double wx = x * grid_resolution_ + map_min_;
        double wy = y * grid_resolution_ + map_min_;
        return Eigen::Vector2d(wx, wy);
    }

    // 获取当前节点的所有邻居节点
    std::vector<Node> getNeighbors(const Node& current) {
        std::vector<Node> neighbors;

        // 八连通邻居
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        for (const auto& dir : directions) {
            int new_x = current.x + dir.first;
            int new_y = current.y + dir.second;

            // 检查新坐标是否在地图范围内
            if(new_x >= 0 && new_x < width_ && new_y >= 0 && new_y < height_){
                // 检查是否为障碍物
                if(grid_map_.grid[new_x][new_y] == 0){
                    double move_cost = (dir.first == 0 || dir.second == 0) ? 1.0 : std::sqrt(2.0); // 直动或斜动
                    neighbors.emplace_back(new_x, new_y, current.g_cost + move_cost, 0.0, nullptr);
                }
            }
        }

        return neighbors;
    }

    // 回溯路径
    std::vector<Eigen::Vector2d> reconstructPath(std::shared_ptr<Node> node) {
        std::vector<Eigen::Vector2d> path;
        while (node) {
            path.push_back(gridToWorld(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        reset();
        return path;
    }

    // 地图数据
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;
    GridMap grid_map_; // 栅格地图，0: 空闲，1: 障碍物
    int num_of_obs_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh;
    double map_min_, map_max_, grid_resolution_;
    double start_x_, start_y_, goal_x_, goal_y_;
    nh.param("astar_planner/map_min", map_min_, -5.0);
    nh.param("astar_planner/map_max", map_max_, 5.0);
    nh.param("astar_planner/grid_resolution", grid_resolution_, 0.1);
    nh.param("astar_planner/start_x", start_x_, -4.5);
    nh.param("astar_planner/start_y", start_y_, -4.5);
    nh.param("astar_planner/goal_x", goal_x_, 4.5);
    nh.param("astar_planner/goal_y", goal_y_, 4.5);

    // 地图参数
    int grid_width = std::round((map_max_ - map_min_) / grid_resolution_);
    int grid_height = grid_width;

    AStarPlanner planner(grid_width, grid_height, map_min_, map_max_, grid_resolution_);
    // 障碍物订阅
    ros::Subscriber obstacle_sub = nh.subscribe<visualization_msgs::MarkerArray>("obstacles", 1,
                                                                                 [&planner, &grid_resolution_, &map_min_](const visualization_msgs::MarkerArray::ConstPtr& msg) {
                                                                                     for (const auto& marker : msg->markers) {
                                                                                         planner.setObstacle(marker.pose.position.x, marker.pose.position.y, marker.scale.x / 2.0);
                                                                                     }
                                                                                 });

    // 发布路径
    ros::Rate rate(10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    // 起点和终点参数
    Eigen::Vector2d start(start_x_, start_y_);
    Eigen::Vector2d goal(goal_x_, goal_y_);
    while (ros::ok()) {
        planner.reset();
        ros::spinOnce();
        // 执行路径搜索
        std::vector<Eigen::Vector2d> path = planner.findPath(start, goal);

        // 路径可视化
        if (path.empty()){
            continue;
        }
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
            path_msg.poses.push_back(pose);
        }
        path_pub.publish(path_msg);
        rate.sleep();
    }

    return 0;
}
