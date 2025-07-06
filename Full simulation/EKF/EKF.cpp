#include "EKF.hpp"
#include <cmath>
#include <iostream>

EKF::EKF() {
    // Initial state (x, y, theta)
    x_.setZero();
    
    // Initial covariance
    P_ = Eigen::Matrix3f::Identity() * 0.1f;
    
    // Process noise (tune these based on your system)
    Q_ = (Eigen::Matrix3f() << 
        0.01f, 0.0f,  0.0f,
        0.0f,  0.01f, 0.0f,
        0.0f,  0.0f,  0.01f).finished();
    
    // Measurement noise for lidar
    R_lidar_ = (Eigen::Matrix3f() << 
        0.05f, 0.0f,  0.0f,
        0.0f,  0.05f, 0.0f,
        0.0f,  0.0f,  0.05f).finished();
    
    // Measurement noise for IMU (pitch/roll and yaw rate)
    R_imu_ = (Eigen::Matrix2f() << 
        0.1f, 0.0f,
        0.0f, 0.1f).finished();
    
    // Initialize IMU biases
    acc_bias_.setZero();
    gyro_bias_.setZero();
}

float EKF::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void EKF::predict(float v, float w, float dt, 
                 const Eigen::Vector3f& imu_acc,
                 const Eigen::Vector3f& imu_gyro) {
    float theta = x_(2);
    
    // Apply IMU bias correction
    Eigen::Vector3f corrected_acc = imu_acc - acc_bias_;
    Eigen::Vector3f corrected_gyro = imu_gyro - gyro_bias_;
    
    // IMU-enhanced prediction (optional - can use just wheel odometry)
    if (imu_acc.norm() > 1e-3 && imu_gyro.norm() > 1e-3) {
        // Fuse IMU and wheel odometry data here
        w = 0.7f * w + 0.3f * corrected_gyro.z(); // Blend yaw rates
    }
    
    // Improved motion model (handles both straight and curved motion)
    if (std::fabs(w) > 1e-5) {
        float R = v / w;
        x_(0) += R * (sin(theta + w * dt) - sin(theta));
        x_(1) += R * (-cos(theta + w * dt) + cos(theta));
    } else {
        x_(0) += v * dt * cos(theta);
        x_(1) += v * dt * sin(theta);
    }
    x_(2) += w * dt;
    x_(2) = normalizeAngle(x_(2));

    // Jacobian of motion model w.r.t. state
    Eigen::Matrix3f F = Eigen::Matrix3f::Identity();
    if (std::fabs(w) > 1e-5) {
        F(0, 2) = (v/w) * (cos(theta + w * dt) - cos(theta));
        F(1, 2) = (v/w) * (sin(theta + w * dt) - sin(theta));
    } else {
        F(0, 2) = -v * dt * sin(theta);
        F(1, 2) = v * dt * cos(theta);
    }

    // Predict covariance
    P_ = F * P_ * F.transpose() + Q_;
}

void EKF::correctLidar(const Eigen::Vector3f& z) {
    Eigen::Vector3f y = z - x_;  // Innovation
    y(2) = normalizeAngle(y(2)); // Normalize angle difference
    
    Eigen::Matrix3f H = Eigen::Matrix3f::Identity();  // Direct measurement
    Eigen::Matrix3f S = H * P_ * H.transpose() + R_lidar_;
    Eigen::Matrix3f K = P_ * H.transpose() * S.inverse();

    // Update state and covariance
    x_ += K * y;
    x_(2) = normalizeAngle(x_(2));
    P_ = (Eigen::Matrix3f::Identity() - K * H) * P_;
}

void EKF::correctIMU(const Eigen::Vector3f& z) {
    // For IMU, we typically only correct orientation (roll/pitch/yaw)
    // and possibly velocity if we're tracking that in the state
    
    // Simplified - just correcting orientation
    Eigen::Vector2f y;
    y << normalizeAngle(z(2) - x_(2)),  // Yaw difference
         0;                             // Placeholder for other corrections
    
    // Measurement matrix (we only measure orientation directly)
    Eigen::Matrix<float, 2, 3> H;
    H << 0, 0, 1,  // Only measures theta
         0, 0, 0;  // Placeholder
    
    Eigen::Matrix2f S = H * P_ * H.transpose() + R_imu_;
    Eigen::Matrix<float, 3, 2> K = P_ * H.transpose() * S.inverse();

    // Update state and covariance
    x_ += K * y;
    x_(2) = normalizeAngle(x_(2));
    P_ = (Eigen::Matrix3f::Identity() - K * H) * P_;
}

void EKF::setIMUNoiseParameters(float acc_noise, float gyro_noise) {
    R_imu_(0, 0) = acc_noise;
    R_imu_(1, 1) = gyro_noise;
}

void EKF::calibrateIMU(const Eigen::Vector3f& acc_bias, const Eigen::Vector3f& gyro_bias) {
    acc_bias_ = acc_bias;
    gyro_bias_ = gyro_bias;
}

Eigen::Vector3f EKF::getState() const { return x_; }
Eigen::Matrix3f EKF::getCovariance() const { return P_; }
