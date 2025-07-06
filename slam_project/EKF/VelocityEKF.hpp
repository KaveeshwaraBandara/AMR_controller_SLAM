#pragma once
#include <Eigen/Dense>

class VelocityEKF {
public:
    using Vector2f = Eigen::Matrix<float, 2, 1>; // [v, w]
    using Matrix2f = Eigen::Matrix<float, 2, 2>;

    VelocityEKF();

    void predict(float dt);
    void correct(const Vector2f &z);  // measurement: [v_icp, w_imu]

    Vector2f getState() const;
    Matrix2f getCovariance() const;

private:
    Vector2f x_;  // state: [v, w]
    Matrix2f P_;
    Matrix2f Q_;  // process noise
    Matrix2f R_;  // measurement noise
};

