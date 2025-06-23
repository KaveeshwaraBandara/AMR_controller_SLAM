#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>

class EKF {
public:
    EKF();
    void predict(float v, float w, float dt);
    void correct(const Eigen::Vector3f& z);

    Eigen::Vector3f getState() const;
    Eigen::Matrix3f getCovariance() const;

private:
    Eigen::Vector3f x_;   // [x, y, theta]
    Eigen::Matrix3f P_;   // Covariance
    Eigen::Matrix3f Q_;   // Process noise
    Eigen::Matrix3f R_;   // Measurement noise
};

#endif

