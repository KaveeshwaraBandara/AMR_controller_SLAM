#include "Astar.hpp"
#include "OccupancyGrid.hpp"

#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <algorithm>

AStarPlanner::AStarPlanner(const OccupancyGrid& grid) : grid_(grid) {}

std::vector<std::pair<int, int>> AStarPlanner::plan(int start_x, int start_y, int goal_x, int goal_y) {
    struct Node {
        int x, y;
        float g, f;
        Node* parent;
        Node(int x, int y, float g, float f, Node* parent)
            : x(x), y(y), g(g), f(f), parent(parent) {}
    };

    auto heuristic = [](int x1, int y1, int x2, int y2) {
        return std::hypot(x1 - x2, y1 - y2);
    };

    auto cmp = [](const Node* a, const Node* b) { return a->f > b->f; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);
    std::unordered_map<int, Node*> visited;

    int width = grid_.getWidth();
    int height = grid_.getHeight();
    auto hash = [&](int x, int y) { return y * width + x; };

    open.push(new Node(start_x, start_y, 0.0f, heuristic(start_x, start_y, goal_x, goal_y), nullptr));

    std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {-1, -1}, {-1, 1}, {1, -1}
    };

    Node* goal_node = nullptr;

    while (!open.empty()) {
        Node* current = open.top(); open.pop();
        int key = hash(current->x, current->y);

        if (visited.count(key)) continue;
        visited[key] = current;

        if (current->x == goal_x && current->y == goal_y) {
            goal_node = current;
            break;
        }

        for (auto [dx, dy] : directions) {
            int nx = current->x + dx;
            int ny = current->y + dy;

            if (!grid_.isInside(nx, ny)) continue;
            if (std::isinf(grid_.getCost(nx, ny))) continue;

            float g_new = current->g + std::hypot(dx, dy);
            float f_new = g_new + heuristic(nx, ny, goal_x, goal_y);

            open.push(new Node(nx, ny, g_new, f_new, current));
        }
    }

    std::vector<std::pair<int, int>> path;
    if (goal_node) {
        Node* node = goal_node;
        while (node) {
            path.emplace_back(node->x, node->y);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
    }

    // Cleanup
    for (auto& [_, node] : visited) delete node;
    return path;
}

void AStarPlanner::drawPath(cv::Mat& image, const std::vector<std::pair<int, int>>& path, const cv::Scalar& color) {
    for (size_t i = 1; i < path.size(); ++i) {
        cv::line(image,
                 cv::Point(path[i - 1].first, path[i - 1].second),
                 cv::Point(path[i].first, path[i].second),
                 color, 1);
    }
}

