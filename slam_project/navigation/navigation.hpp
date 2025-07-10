#pragma once
#include "OccupancyGrid.hpp"
#include "Astar.hpp"
#include "PoseEKF.hpp"
#include "VelocityEKF.hpp"
#include "send.hpp"
#include "BNO055.hpp"
#include "LidarReader.hpp"
#include <Eigen/Dense>
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
	Pose2D current_pose;
    void setGoal(float gx, float gy);
    void navigateToGoal();  // <-- Step 2 here
	int goal_x_, goal_y_;
    PoseEKF::Vector3f statee;

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
    //int goal_x_, goal_y_;
    bool goal_set_ = false;

    std::vector<std::pair<float, float>> convertPathToWorld(const std::vector<std::pair<int, int>>& path);
    void computeAndSendCommand(float curr_x, float curr_y, float curr_theta,
                               float target_x, float target_y);
};
