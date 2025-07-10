#include "Trajviz.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

void TrajectoryVisualizer::drawTrajectoryOnMap(const std::string& map_path,
                                               const std::vector<Pose2D>& trajectory,
                                               const OccupancyGrid& grid,
                                               const std::pair<int, int>& goal_grid,
                                               const std::string& output_path) {
    cv::Mat map = cv::imread(map_path, cv::IMREAD_COLOR);
    if (map.empty()) {
        std::cerr << "Failed to load map image: " << map_path << "\n";
        return;
    }

    for (size_t i = 1; i < trajectory.size(); ++i) {
        const auto& prev = trajectory[i - 1];
        const auto& curr = trajectory[i];

        int x0 = static_cast<int>(prev.x / grid.getResolution()) + grid.getOriginX();
        int y0 = static_cast<int>(prev.y / grid.getResolution()) + grid.getOriginY();
        int x1 = static_cast<int>(curr.x / grid.getResolution()) + grid.getOriginX();
        int y1 = static_cast<int>(curr.y / grid.getResolution()) + grid.getOriginY();

        cv::line(map, {x0, y0}, {x1, y1}, cv::Scalar(0, 0, 255), 1); // red
    }

    if (!trajectory.empty()) {
        const auto& last = trajectory.back();
        int x = static_cast<int>(last.x / grid.getResolution()) + grid.getOriginX();
        int y = static_cast<int>(last.y / grid.getResolution()) + grid.getOriginY();
        float theta = last.theta;

        int arrow_len = 10;
        int x2 = x + static_cast<int>(arrow_len * std::cos(theta));
        int y2 = y + static_cast<int>(arrow_len * std::sin(theta));
        cv::arrowedLine(map, {x, y}, {x2, y2}, cv::Scalar(0, 255, 255), 2); // yellow
    }

    cv::circle(map, cv::Point(goal_grid.first, goal_grid.second), 5, cv::Scalar(0, 255, 0), -1);

    cv::imwrite(output_path, map);
    std::cout << "[Info] Saved map with trajectory: " << output_path << "\n";
}
