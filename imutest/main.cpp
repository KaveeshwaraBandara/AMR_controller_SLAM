#include "BNO055.hpp"
#include <iostream>
#include <unistd.h>

int main() {
    const uint8_t IMU_ADDR = 0x28;  // Renamed to avoid macro conflict

    BNO055 imu(IMU_ADDR);

    if (!imu.begin()) {
        std::cerr << "Failed to initialize BNO055 sensor!" << std::endl;
        return 1;
    }

    std::cout << "BNO055 initialized and calibrated.\n";

    while (true) {
        EulerAngles eul = imu.readEulerAngles();
        Vector3 gyro = imu.readAngularVelocity();
        Vector3 accel = imu.readLinearAcceleration();
        Vector3 gravity = imu.readGravity();

        std::cout << "Euler Angles [Yaw: " << eul.yaw
                  << ", Roll: " << eul.roll
                  << ", Pitch: " << eul.pitch << "]" << std::endl;

        std::cout << "Gyro [x: " << gyro.x
                  << ", y: " << gyro.y
                  << ", z: " << gyro.z << "] deg/s" << std::endl;

        std::cout << "Linear Accel [x: " << accel.x
                  << ", y: " << accel.y
                  << ", z: " << accel.z << "] m/s^2" << std::endl;

        std::cout << "Gravity [x: " << gravity.x
                  << ", y: " << gravity.y
                  << ", z: " << gravity.z << "] m/s^2" << std::endl;

        std::cout << "-------------------------" << std::endl;

        usleep(500000); // 500 ms
    }

    return 0;
}

