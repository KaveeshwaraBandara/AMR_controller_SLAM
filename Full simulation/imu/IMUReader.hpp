#ifndef IMUREADER_HPP
#define IMUREADER_HPP

#include <Eigen/Dense>
#include <mutex>
#include <vector>

class IMU {
public:
    struct Data {
        double timestamp;
        Eigen::Vector3f linear_acceleration;
        Eigen::Vector3f angular_velocity;
        Eigen::Vector3f orientation; // Roll, pitch, yaw in radians
    };

    IMU();
    
    // Update IMU data
    void update(const Data& new_data);
    
    // Get filtered IMU data
    Data getFilteredData() const;
    
    // Get orientation (roll, pitch, yaw)
    Eigen::Vector3f getOrientation() const;
    
    // Get angular velocity
    Eigen::Vector3f getAngularVelocity() const;
    
    // Get linear acceleration
    Eigen::Vector3f getLinearAcceleration() const;

private:
    mutable std::mutex data_mutex_;
    Data current_data_;
    
    // Low-pass filter parameters
    const float alpha_ = 0.2f; // Smoothing factor
    
    // Apply low-pass filter
    void applyFilter(const Data& new_data);
};

#endif // IMU_HPP
