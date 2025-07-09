// navigation/Navigation.cpp
#include "Navigation.hpp"
#include <cmath>
#include <iostream>
#include <unistd.h>

Navigator::Navigator(OccupancyGrid& g, AStarPlanner& p, PoseEKF& ekf, SerialPort& s)
    : grid(g), planner(p), poseEKF(ekf), serial(s) {
    resolution = grid.getResolution();
    originX = grid.getOriginX();
    originY = grid.getOriginY();
}

void Navigator::setGoal(float gx, float gy) {
    goal_x = gx;
    goal_y = gy;
}

std::vector<std::pair<int, int>> Navigator::planPath() {
    int goal_cx = static_cast<int>(goal_x / resolution) + originX;
    int goal_cy = static_cast<int>(goal_y / resolution) + originY;

    auto state = poseEKF.getState();
    int start_cx = static_cast<int>(state(0) / resolution) + originX;
    int start_cy = static_cast<int>(state(1) / resolution) + originY;

    return planner.plan(start_cx, start_cy, goal_cx, goal_cy);
}

void Navigator::sendVelocityToTarget(float robot_x, float robot_y, float robot_theta,
                                     float target_x, float target_y, float dt) {
    float dx = target_x - robot_x;
    float dy = target_y - robot_y;
    float distance = std::sqrt(dx * dx + dy * dy);
    float heading = std::atan2(dy, dx);
    float angle_diff = heading - robot_theta;

    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    float v = 0.03f;
    float w = angle_diff;

    if (w > 1.0f) w = 1.0f;
    if (w < -1.0f) w = -1.0f;

    serial.sendCommand(v, w);
    usleep(dt * 1e6);

    poseEKF.predict(v, w, dt);
}

void Navigator::runNavigationLoop() {
    while (true) {
        auto path = planPath();
        if (path.empty()) {
            std::cerr << "No path found.\n";
            break;
        }

        auto state = poseEKF.getState();
        float robot_x = state(0);
        float robot_y = state(1);
        float robot_theta = state(2);

        int index = std::min(stepAhead, static_cast<int>(path.size() - 1));
        auto [cx, cy] = path[index];
        float target_x = (cx - originX) * resolution;
        float target_y = (cy - originY) * resolution;

        sendVelocityToTarget(robot_x, robot_y, robot_theta, target_x, target_y, 0.3f);

        float goal_dist = std::hypot((goal_x - robot_x), (goal_y - robot_y));
        if (goal_dist < distanceThreshold) {
            std::cout << "Reached goal.\n";
            break;
        }
    }

    serial.sendCommand(0, 0); // Stop robot
}
