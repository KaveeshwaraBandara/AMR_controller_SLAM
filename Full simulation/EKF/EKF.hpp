#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>

class EKF {
public:
    EKF();
    
    // Prediction with both control inputs and IMU data
    void predict(float v, float w, float dt, 
                 const Eigen::Vector3f& imu_acc = Eigen::Vector3f::Zero(),
                 const Eigen::Vector3f& imu_gyro = Eigen::Vector3f::Zero());
    
    // Correction with different measurement types
    void correctLidar(const Eigen::Vector3f& z);
    void correctIMU(const Eigen::Vector3f& z);
    
    // State accessors
    Eigen::Vector3f getState() const;
    Eigen::Matrix3f getCovariance() const;
    
    // IMU-specific functions
    void setIMUNoiseParameters(float acc_noise, float gyro_noise);
    void calibrateIMU(const Eigen::Vector3f& acc_bias, const Eigen::Vector3f& gyro_bias);
    void setState(const Eigen::Vector3f& x) { x_ = x; }


private:
    Eigen::Vector3f x_;  // State: [x, y, theta]^T
    Eigen::Matrix3f P_;  // Covariance matrix
    Eigen::Matrix3f Q_;  // Process noise covariance
    Eigen::Matrix3f R_lidar_;  // Lidar measurement noise
    Eigen::Matrix2f R_imu_;    // IMU measurement noise
    
    // IMU calibration parameters
    Eigen::Vector3f acc_bias_;
    Eigen::Vector3f gyro_bias_;
    
    // Normalize angle to [-pi, pi]
    float normalizeAngle(float angle);
};

#endif // EKF_HPP
