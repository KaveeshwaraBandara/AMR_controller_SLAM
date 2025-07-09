// navigation/Navigator.hpp

#pragma once

#include "OccupancyGrid.hpp"
#include "Astar.hpp"
#include "PoseEKF.hpp"
#include "SerialPort.hpp"
#include <vector>
#include <opencv2/opencv.hpp>

class Navigator {
public:
    Navigator(OccupancyGrid& grid, PoseEKF& ekf, SerialPort& serial);

    void setGoal();                     // User clicks on cost map
    void setGoal(int gx, int gy);       // Optional manual setting

    void runNavigationLoop();

private:
    OccupancyGrid& grid_;
    PoseEKF& poseEKF_;
    SerialPort& serial_;
    AStarPlanner planner_;

    int goal_x_, goal_y_;
    bool goal_set_ = false;

    void computeAndSendCommand(int curr_x, int curr_y, int goal_x, int goal_y, const std::vector<std::pair<int, int>>& path);
    float normalizeAngle(float angle);
};

