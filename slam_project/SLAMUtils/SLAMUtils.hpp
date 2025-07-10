#pragma once

#include "VelocityEKF.hpp"
#include "PoseEKF.hpp"
#include "OccupancyGrid.hpp"
#include "BNO055.hpp"
#include <opencv2/core.hpp>
#include <vector>
#include <utility>
#include <thread>

void updateSLAM(const std::vector<std::pair<float, float>>& scan,
                const EulerAngles& orientation,
                const Vector3& angularVel,
                std::vector<cv::Point2f>& prev_cloud,
                VelocityEKF& velocityEKF,
                PoseEKF& poseEKF,
                std::vector<Pose2D>& trajectory,
                OccupancyGrid& grid,
                float dt,Pose2D& current_pose,PoseEKF::Vector3f& statee);
