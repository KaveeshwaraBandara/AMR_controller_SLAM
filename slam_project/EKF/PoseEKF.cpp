#include "PoseEKF.hpp"
#include <cmath>

PoseEKF::PoseEKF() {
    x_.setZero();
    P_ = Matrix3f::Identity() * 0.1f;
    Q_ = Matrix3f::Identity() * 0.01f;
    R_ = Matrix3f::Identity() * 0.05f;
}

void PoseEKF::predict(float v, float w, float dt) {
    float theta = x_(2);

    if (std::fabs(w) > 1e-5) {
        float R = v / w;
        x_(0) += R * (std::sin(theta + w * dt) - std::sin(theta));
        x_(1) += R * (-std::cos(theta + w * dt) + std::cos(theta));
    } else {
        x_(0) += v * dt * std::cos(theta);
        x_(1) += v * dt * std::sin(theta);
    }
    x_(2) += w * dt;

    // Normalize angle
    //x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));

    // Jacobian F
    Matrix3f F = Matrix3f::Identity();
    F(0, 2) = -v * dt * std::sin(theta);
    F(1, 2) =  v * dt * std::cos(theta);

    P_ = F * P_ * F.transpose() + Q_;
}

void PoseEKF::correct(const Vector3f &z) {
    Vector3f y = z - x_;
    y(2) = std::atan2(std::sin(y(2)), std::cos(y(2)));

    Matrix3f H = Matrix3f::Identity();

    Matrix3f S = H * P_ * H.transpose() + R_;
    Matrix3f K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Matrix3f::Identity() - K * H) * P_;
}

PoseEKF::Vector3f PoseEKF::getState() const {
    return x_;
}

PoseEKF::Matrix3f PoseEKF::getCovariance() const {
    return P_;
}

