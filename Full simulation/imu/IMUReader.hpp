#ifndef IMU_READER_HPP
#define IMU_READER_HPP

class IMUReader {
public:
    IMUReader();
    bool initialize();
    void update();

    float getLinearVelocity();   // in m/s
    float getAngularVelocity();  // in rad/s

private:
    float v_;  // estimated linear velocity
    float w_;  // estimated angular velocity
    // Add your IMU reading logic and buffer if needed
};

#endif

