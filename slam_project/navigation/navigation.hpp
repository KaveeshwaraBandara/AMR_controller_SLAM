// navigation/Navigation.hpp
#pragma once
#include "OccupancyGrid.hpp"
#include "Astar.hpp"
#include "PoseEKF.hpp"
#include "send.hpp"

class Navigator {
public:
    Navigator(OccupancyGrid& grid, AStarPlanner& planner, PoseEKF& poseEKF, SerialPort& serial);

    void setGoal(float gx, float gy);  // world coordinates in meters
    void runNavigationLoop();

private:
    OccupancyGrid& grid;
    AStarPlanner& planner;
    PoseEKF& poseEKF;
    SerialPort& serial;
    
    float goal_x, goal_y;
    float resolution;
    int originX, originY;

    std::vector<std::pair<int, int>> planPath();
    void sendVelocityToTarget(float robot_x, float robot_y, float robot_theta,
                              float target_x, float target_y, float dt);

    float distanceThreshold = 0.2f;
    int stepAhead = 4;
};
