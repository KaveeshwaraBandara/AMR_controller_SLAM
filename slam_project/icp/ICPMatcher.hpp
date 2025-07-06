#pragma once
#include <vector>
#include <Eigen/Dense>

// ICP using KNN with OpenCV but outputs Eigen
Eigen::Matrix3f runICP(const std::vector<Eigen::Vector2f>& ref,
                       const std::vector<Eigen::Vector2f>& target);

