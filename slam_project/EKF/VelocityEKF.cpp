#include "VelocityEKF.hpp"

VelocityEKF::VelocityEKF() {
    x_.setZero();
    P_ = Matrix2f::Identity() * 0.1f;
    Q_ = Matrix2f::Identity() * 0.01f;
    R_ = Matrix2f::Identity() * 0.05f;
}

void VelocityEKF::predict(float dt) {
    // State doesn't change without control input, so prediction is x = x
    // Just increase uncertainty
    P_ = P_ + Q_;
}

void VelocityEKF::correct(const Vector2f &z) {
    // Innovation
    Vector2f y = z - x_;

    // Measurement matrix is identity (direct observation)
    Matrix2f H = Matrix2f::Identity();

    Matrix2f S = H * P_ * H.transpose() + R_;
    Matrix2f K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Matrix2f::Identity() - K * H) * P_;
}

VelocityEKF::Vector2f VelocityEKF::getState() const {
    return x_;
}

VelocityEKF::Matrix2f VelocityEKF::getCovariance() const {
    return P_;
}

