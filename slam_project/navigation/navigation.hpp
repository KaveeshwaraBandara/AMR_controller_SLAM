#pragma once
#include "OccupancyGrid.hpp"
#include "Astar.hpp"
#include "PoseEKF.hpp"
#include "VelocityEKF.hpp"
#include "send.hpp"
#include "BNO055.hpp"
#include "LidarReader.hpp"
#include <opencv2/opencv.hpp>

class Navigator {
public:
Navigator(OccupancyGrid& grid,
    AStarPlanner& planner,
    PoseEKF& poseEKF,
    VelocityEKF& velocityEKF,
    BNO055& imu,
    LidarReader& lidar,
    std::vector<Pose2D>& trajectory,
    std::vector<cv::Point2f>& prev_cloud,
    SerialPort& serial);
    int goal_x_, goal_y_;
    void setGoal(int gx, int gy);
    void navigateToGoal();  // <-- Step 2 here

private:
OccupancyGrid& grid_;
AStarPlanner& planner_;
PoseEKF& poseEKF_;
VelocityEKF& velocityEKF_;
BNO055& imu_;
LidarReader& lidar_;
std::vector<Pose2D>& trajectory_;
std::vector<cv::Point2f>& prev_cloud_;
SerialPort& serial_;
    
    bool goal_set_ = false;

    std::vector<std::pair<float, float>> convertPathToWorld(const std::vector<std::pair<int, int>>& path);
    void computeAndSendCommand(float curr_x, float curr_y, float curr_theta,
                               float target_x, float target_y);
};