#include "navigation.hpp"
#include <cmath>
#include <unistd.h>
#include <iostream>

Navigator::Navigator(OccupancyGrid& grid, AStarPlanner& planner, PoseEKF& ekf, SerialPort& serial)
    : grid_(grid), planner_(planner), poseEKF_(ekf), serial_(serial) {}

void Navigator::setGoal(int gx, int gy) {
    goal_x_ = static_cast<int>(gx / grid_.getResolution()) + grid_.getOriginX();
    goal_y_ = static_cast<int>(gy / grid_.getResolution()) + grid_.getOriginY();
    //goal_y_ = gy;
    goal_set_ = true;
}

std::vector<std::pair<float, float>> Navigator::convertPathToWorld(const std::vector<std::pair<int, int>>& path) {
    std::vector<std::pair<float, float>> world_path;
    for (auto [x, y] : path) {
        float wx = (x - grid_.getOriginX()) * grid_.getResolution();
        float wy = (y - grid_.getOriginY()) * grid_.getResolution();
        world_path.emplace_back(wx, wy);
    }
    return world_path;
}

void Navigator::computeAndSendCommand(float curr_x, float curr_y, float curr_theta,
                                      float target_x, float target_y) {
    float dx = target_x - curr_x;
    float dy = target_y - curr_y;
    float dist = std::sqrt(dx * dx + dy * dy);
    float target_heading = std::atan2(dy, dx);
    float angle_diff = target_heading - curr_theta;

    // Normalize angle to [-π, π]
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    float v = std::min(0.05f, dist);  // Limit forward speed
    float w = angle_diff;

    // Clamp angular speed
    if (w > 1.0f) w = 1.0f;
    if (w < -1.0f) w = -1.0f;

    serial_.sendCommand(v, w);
}

void Navigator::navigateToGoal() {
    if (!goal_set_) {
        std::cerr << "[Navigator] No goal set.\n";
        return;
    }

    auto state = poseEKF_.getState();
    int start_x = static_cast<int>(state(0) / grid_.getResolution()) + grid_.getOriginX();
    int start_y = static_cast<int>(state(1) / grid_.getResolution()) + grid_.getOriginY();

    auto path = planner_.plan(start_x, start_y, goal_x_, goal_y_);
    if (path.empty()) {
        std::cerr << "[Navigator] Path planning failed.\n";
        return;
    }

    auto world_path = convertPathToWorld(path);

    for (auto [tx, ty] : world_path) {
        auto pose = poseEKF_.getState();
        float curr_x = pose(0), curr_y = pose(1), curr_theta = pose(2);

        float dist = std::hypot(curr_x - tx, curr_y - ty);
        if (dist < 0.1f) continue;

        computeAndSendCommand(curr_x, curr_y, curr_theta, tx, ty);

        usleep(300000);  // Wait for motion
    }

    serial_.sendCommand(0.0, 0.0);  // Stop
    std::cout << "[Navigator] Reached goal (coarse mode).\n";
}