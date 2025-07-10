#pragma once
#include "OccupancyGrid.hpp"
#include "Astar.hpp"
#include "PoseEKF.hpp"
#include "send.hpp"

class Navigator {
public:
    Navigator(OccupancyGrid& grid, AStarPlanner& planner, PoseEKF& ekf, SerialPort& serial);

    void setGoal(int gx, int gy);
    void navigateToGoal();  // <-- Step 2 here

private:
    OccupancyGrid& grid_;
    AStarPlanner& planner_;
    PoseEKF& poseEKF_;
    SerialPort& serial_;
    int goal_x_, goal_y_;
    bool goal_set_ = false;

    std::vector<std::pair<float, float>> convertPathToWorld(const std::vector<std::pair<int, int>>& path);
    void computeAndSendCommand(float curr_x, float curr_y, float curr_theta,
                               float target_x, float target_y);
};