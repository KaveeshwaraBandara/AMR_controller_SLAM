#pragma once

#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>

class OccupancyGrid; // Forward declaration

class AStarPlanner {
public:
    AStarPlanner(const OccupancyGrid& grid);

    // Compute path from start to goal in grid coordinates
    std::vector<std::pair<int, int>> plan(int start_x, int start_y, int goal_x, int goal_y);

    // Optional: Draw path on an image
    static void drawPath(cv::Mat& image, const std::vector<std::pair<int, int>>& path, const cv::Scalar& color);

private:
    const OccupancyGrid& grid_;
};

