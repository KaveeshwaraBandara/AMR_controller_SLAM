// navigation/Navigator.hpp

#pragma once

#include "OccupancyGrid.hpp"
#include "Astar.hpp"
#include "PoseEKF.hpp"
#include "send.hpp"
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>

class Navigator {
public:
    Navigator(OccupancyGrid& grid, PoseEKF& ekf, SerialPort& serial);

    void setGoal();                     // User clicks on cost map
    void setGoal(int gx, int gy);       // Optional manual setting

    void runNavigationLoop();
    //std::vector<cv::Point2f> toPointCloud(const std::vector<std::pair<float, float>>& scan);
    //std::vector<std::pair<float, float>> transformToGlobal(const std::vector<std::pair<float, float>>& scan, const Pose2D& pose);
    
std::vector<std::pair<float, float>> transformToGlobal(const std::vector<std::pair<float, float>>& scan, const Pose2D& pose) {
    std::vector<std::pair<float, float>> global;
    float c = cos(pose.theta), s = sin(pose.theta);
    for (auto [angle_deg, dist_m] : scan) {
        float theta = angle_deg * CV_PI / 180.0f;
        float x = dist_m * cos(theta), y = dist_m * sin(theta);
        float gx = x * c - y * s + pose.x;
        float gy = x * s + y * c + pose.y;
        global.emplace_back(gx, gy);
    }
    return global;
}

private:
    OccupancyGrid& grid_;
    PoseEKF& poseEKF_;
    SerialPort& serial_;
    AStarPlanner planner_;

    int goal_x_, goal_y_;
    bool goal_set_ = false;

    void computeAndSendCommand(int curr_x, int curr_y, int goal_x, int goal_y, const std::vector<std::pair<int, int>>& path);
    float normalizeAngle(float angle);
     void updatePoseFromSLAM();
};

