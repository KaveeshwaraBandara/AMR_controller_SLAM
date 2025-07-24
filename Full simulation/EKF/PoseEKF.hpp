#pragma once
#include <Eigen/Dense>

class PoseEKF {
public:
    using Vector3f = Eigen::Matrix<float, 3, 1>; // [x, y, theta]
    using Matrix3f = Eigen::Matrix<float, 3, 3>;

    PoseEKF();

    void predict(float v, float w, float dt);
    void correct(const Vector3f &z);  // ICP pose: [x, y, theta]

    Vector3f getState() const;
    Matrix3f getCovariance() const;

private:
    Vector3f x_;  // pose state: [x, y, theta]
    Matrix3f P_;
    Matrix3f Q_;
    Matrix3f R_;
};

