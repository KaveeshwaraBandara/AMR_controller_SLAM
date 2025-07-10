#pragma once

#include "VelocityEKF.hpp"
#include "PoseEKF.hpp"
#include "OccupancyGrid.hpp"
#include "BNO055.hpp"
#include <opencv2/core.hpp>
#include <vector>
#include <utility>
#include <thread>
#include "navigation.hpp"

void updateSLAM(const std::vector<std::pair<float, float>>& scan,
                const EulerAngles& orientation,
                const Vector3& angularVel,
                std::vector<cv::Point2f>& prev_cloud,
                VelocityEKF& velocityEKF,
                PoseEKF& poseEKF,
                std::vector<Pose2D>& trajectory,
                OccupancyGrid& grid,
                float dt);
                
                void drawNavigationDebug(const OccupancyGrid& grid,
                    const std::vector<std::pair<int, int>>& path,
                    const Pose2D& robot_pose,
                    int goal_x, int goal_y);