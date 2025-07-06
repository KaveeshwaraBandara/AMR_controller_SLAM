#include "IMUReader.hpp"
#include <iostream>

IMU::IMU() {
    current_data_ = {
        0.0,
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero()
    };
}

void IMU::update(const Data& new_data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    applyFilter(new_data);
}

void IMU::applyFilter(const Data& new_data) {
    // Simple low-pass filter implementation
    current_data_.linear_acceleration = 
        alpha_ * new_data.linear_acceleration + 
        (1 - alpha_) * current_data_.linear_acceleration;
        
    current_data_.angular_velocity = 
        alpha_ * new_data.angular_velocity + 
        (1 - alpha_) * current_data_.angular_velocity;
        
    current_data_.orientation = 
        alpha_ * new_data.orientation + 
        (1 - alpha_) * current_data_.orientation;
        
    current_data_.timestamp = new_data.timestamp;
}

IMU::Data IMU::getFilteredData() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_data_;
}

Eigen::Vector3f IMU::getOrientation() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_data_.orientation;
}

Eigen::Vector3f IMU::getAngularVelocity() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_data_.angular_velocity;
}

Eigen::Vector3f IMU::getLinearAcceleration() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_data_.linear_acceleration;
}
