// navigation/Navigator.cpp

#include "Navigator.hpp"
#include <cmath>
#include <unistd.h>
#include <iostream>

Navigator::Navigator(OccupancyGrid& grid, PoseEKF& ekf, SerialPort& serial)
    : grid_(grid), poseEKF_(ekf), serial_(serial), planner_(grid) {}

void Navigator::setGoal() {
    grid_.showCostMapWithGoalSelection();
    if (grid_.isGoalSelected()) {
        std::tie(goal_x_, goal_y_) = grid_.getSelectedGoal();
        goal_set_ = true;
        std::cout << "[Navigator] Goal selected at (" << goal_x_ << ", " << goal_y_ << ")\n";
    } else {
        std::cerr << "[Navigator] No goal selected.\n";
        goal_set_ = false;
    }
}

void Navigator::setGoal(int gx, int gy) {
    goal_x_ = gx;
    goal_y_ = gy;
    goal_set_ = true;
}

float Navigator::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void Navigator::computeAndSendCommand(int curr_x, int curr_y, int goal_x, int goal_y, const std::vector<std::pair<int, int>>& path) {
    if (path.size() < 2) return;

    // Find a lookahead point
    std::pair<int, int> target = path[std::min<size_t>(4, path.size() - 1)];

    float target_x_m = (target.first - grid_.getOriginX()) * grid_.getResolution();
    float target_y_m = (target.second - grid_.getOriginY()) * grid_.getResolution();

    auto state = poseEKF_.getState();
    float curr_x_m = state(0);
    float curr_y_m = state(1);
    float theta = state(2);

    float dx = target_x_m - curr_x_m;
    float dy = target_y_m - curr_y_m;
    float heading = std::atan2(dy, dx);
    float angle_diff = normalizeAngle(heading - theta);
    float distance = std::sqrt(dx * dx + dy * dy);

    float v = std::min(0.03f, distance);  // Optional: scale velocity by distance
    float w = angle_diff;

    // Clamp angular velocity
    if (w > 1.0f) w = 1.0f;
    if (w < -1.0f) w = -1.0f;

    serial_.sendCommand(v, w);
}

void Navigator::runNavigationLoop() {
    if (!goal_set_) {
        std::cerr << "[Navigator] No goal set. Aborting.\n";
        return;
    }

    while (true) {
        auto state = poseEKF_.getState();
        int curr_x = static_cast<int>(state(0) / grid_.getResolution()) + grid_.getOriginX();
        int curr_y = static_cast<int>(state(1) / grid_.getResolution()) + grid_.getOriginY();

        auto path = planner_.plan(curr_x, curr_y, goal_x_, goal_y_);
        if (path.empty()) {
            std::cerr << "[Navigator] Path planning failed.\n";
            serial_.sendCommand(0.0, 0.0);
            break;
        }

        computeAndSendCommand(curr_x, curr_y, goal_x_, goal_y_, path);

        float goal_x_m = (goal_x_ - grid_.getOriginX()) * grid_.getResolution();
        float goal_y_m = (goal_y_ - grid_.getOriginY()) * grid_.getResolution();
        float dist_to_goal = std::hypot(state(0) - goal_x_m, state(1) - goal_y_m);

        if (dist_to_goal < 0.15f) {
            std::cout << "[Navigator] Goal reached!\n";
            serial_.sendCommand(0.0, 0.0);
            break;
        }

        usleep(300000); // 300 ms delay between steps
    }
}

