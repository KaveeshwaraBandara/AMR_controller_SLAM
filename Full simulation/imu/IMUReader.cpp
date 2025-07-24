#include "IMUReader.hpp"
#include <cmath>

IMUReader::IMUReader() : v_(0.0f), w_(0.0f) {}

bool IMUReader::initialize() {
    // Setup I2C or Serial communication with IMU
    return true;
}

void IMUReader::update() {
    // Replace with actual sensor code
    // Example: integrate acceleration to velocity
    v_ = 0.2f;  // placeholder
    w_ = 0.05f; // placeholder
}

float IMUReader::getLinearVelocity()  { return v_; }
float IMUReader::getAngularVelocity() { return w_; }

