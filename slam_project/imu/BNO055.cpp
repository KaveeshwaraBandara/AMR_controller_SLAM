#include "BNO055.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cmath>

#define I2C_BUS "/dev/i2c-1"

static int i2c_fd;

BNO055::BNO055(uint8_t address) : i2c_address(address) {}

bool BNO055::begin() {
    i2c_fd = open(I2C_BUS, O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "Cannot open I2C bus\n";
        return false;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, i2c_address) < 0) {
        std::cerr << "Failed to connect to BNO055\n";
        return false;
    }

    // Set to config mode
    write8(BNO055_OPR_MODE_ADDR, CONFIGMODE);
    usleep(20000);

    // Set to NDOF mode
    write8(BNO055_OPR_MODE_ADDR, NDOF);
    usleep(20000);
    
   // waitForCalibration();  //i just add this for ensure the calibration   , but i see in NDOF fusion method there is auto calibration 
    
    return true;
}

EulerAngles BNO055::readEulerAngles() {
    uint8_t buffer[6];
    readLen(BNO055_EULER_H_LSB_ADDR, buffer, 6);

    int16_t h = (buffer[1] << 8) | buffer[0];
    int16_t r = (buffer[3] << 8) | buffer[2];
    int16_t p = (buffer[5] << 8) | buffer[4];

    EulerAngles eul;
    eul.yaw = h / 16.0f;
    eul.roll = r / 16.0f;
    eul.pitch = p / 16.0f;
    return eul;
}

Vector3 BNO055::readAngularVelocity() {
    uint8_t buffer[6];
    readLen(BNO055_GYRO_DATA_ADDR, buffer, 6);

    Vector3 gyro;
    gyro.x = ((int16_t)(buffer[1] << 8 | buffer[0])) / 16.0f;
    gyro.y = ((int16_t)(buffer[3] << 8 | buffer[2])) / 16.0f;
    gyro.z = ((int16_t)(buffer[5] << 8 | buffer[4])) / 16.0f;
    return gyro;
}

Vector3 BNO055::readLinearAcceleration() {
    uint8_t buffer[6];
    readLen(BNO055_LINEAR_ACCEL_DATA_ADDR, buffer, 6);

    Vector3 accel;
    accel.x = ((int16_t)(buffer[1] << 8 | buffer[0])) / 100.0f;
    accel.y = ((int16_t)(buffer[3] << 8 | buffer[2])) / 100.0f;
    accel.z = ((int16_t)(buffer[5] << 8 | buffer[4])) / 100.0f;
    return accel;
}

Vector3 BNO055::readGravity() {
    uint8_t buffer[6];
    readLen(BNO055_GRAVITY_DATA_ADDR, buffer, 6);

    Vector3 g;
    g.x = ((int16_t)(buffer[1] << 8 | buffer[0])) / 100.0f;
    g.y = ((int16_t)(buffer[3] << 8 | buffer[2])) / 100.0f;
    g.z = ((int16_t)(buffer[5] << 8 | buffer[4])) / 100.0f;
    return g;
}

// I2C low-level functions
void BNO055::write8(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    write(i2c_fd, buffer, 2);
}

uint8_t BNO055::read8(uint8_t reg) {
    write(i2c_fd, &reg, 1);
    uint8_t value;
    read(i2c_fd, &value, 1);
    return value;
}

void BNO055::readLen(uint8_t reg, uint8_t* buffer, uint8_t len) {
    write(i2c_fd, &reg, 1);
    read(i2c_fd, buffer, len);
}

bool BNO055::waitForCalibration() {
    std::cout << "Waiting for full BNO055 calibration..." << std::endl;
    while (true) {
        uint8_t calib = read8(0x35);
        uint8_t sys = (calib >> 6) & 0x03;
        uint8_t gyr = (calib >> 4) & 0x03;
        uint8_t acc = (calib >> 2) & 0x03;
        uint8_t mag = (calib) & 0x03;

        std::cout << "Calib → SYS: " << +sys << ", GYR: " << +gyr
                  << ", ACC: " << +acc << ", MAG: " << +mag << "\r";
        std::cout.flush();

        if (sys == 3 && gyr == 3 && acc == 3 && mag == 3)
            break;

        usleep(500000); // 500ms
    }

    std::cout << "\nCalibration complete.\n";
    return true;
}

