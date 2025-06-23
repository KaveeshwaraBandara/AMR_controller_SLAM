#include "EKF.hpp"
#include <cmath>

EKF::EKF() {
    x_.setZero();
    P_ = Eigen::Matrix3f::Identity() * 0.1f;
    Q_ = Eigen::Matrix3f::Identity() * 0.01f;
    R_ = Eigen::Matrix3f::Identity() * 0.05f;
}

void EKF::predict(float v, float w, float dt) {
    float theta = x_(2);

    // Motion model
    /*x_(0) += v * dt * std::cos(theta);
    x_(1) += v * dt * std::sin(theta);
    x_(2) += w * dt;*/
    
    if (std::fabs(w) > 1e-5) {
    float R = v / w;
    x_(0) += R * (sin(theta + w * dt) - sin(theta));
    x_(1) += R * (-cos(theta + w * dt) + cos(theta));
    } else {
      x_(0) += v * dt * cos(theta);
      x_(1) += v * dt * sin(theta);
    }
      x_(2) += w * dt;


    // Jacobian w.r.t. state
    Eigen::Matrix3f F = Eigen::Matrix3f::Identity();
    F(0, 2) = -v * dt * std::sin(theta);
    F(1, 2) =  v * dt * std::cos(theta);

    // Predict covariance
    P_ = F * P_ * F.transpose() + Q_;
}

void EKF::correct(const Eigen::Vector3f& z) {
    Eigen::Vector3f y = z - x_;  // Innovation
    y(2) = std::atan2(std::sin(y(2)), std::cos(y(2))); // angle normalization

    Eigen::Matrix3f H = Eigen::Matrix3f::Identity();  // Identity for direct measurement
    Eigen::Matrix3f S = H * P_ * H.transpose() + R_;
    Eigen::Matrix3f K = P_ * H.transpose() * S.inverse();

    x_ += K * y;
    P_ = (Eigen::Matrix3f::Identity() - K * H) * P_;
}

Eigen::Vector3f EKF::getState() const { return x_; }
Eigen::Matrix3f EKF::getCovariance() const { return P_; }

