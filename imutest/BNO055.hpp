#ifndef BNO055_HPP
#define BNO055_HPP

#include <cstdint>
#include <array>

// Default I2C address for BNO055
#define BNO055_ADDRESS 0x28

// Operation Modes
enum BNO055OpMode {
    CONFIGMODE = 0x00,
    NDOF       = 0x0C,
};

// BNO055 Registers
enum BNO055Registers {
    BNO055_CHIP_ID_ADDR = 0x00,
    BNO055_OPR_MODE_ADDR = 0x3D,
    BNO055_SYS_TRIGGER_ADDR = 0x3F,

    BNO055_EULER_H_LSB_ADDR = 0x1A,
    BNO055_GYRO_DATA_ADDR = 0x14,
    BNO055_LINEAR_ACCEL_DATA_ADDR = 0x28,
    BNO055_GRAVITY_DATA_ADDR = 0x2E
};

struct EulerAngles {
    float yaw;
    float roll;
    float pitch;
};

struct Vector3 {
    float x;
    float y;
    float z;
};

class BNO055 {
public:
    explicit BNO055(uint8_t address = BNO055_ADDRESS);

    bool begin();                         // Initialize sensor in NDOF mode
    EulerAngles readEulerAngles();       // In degrees
    Vector3 readAngularVelocity();       // In deg/sec
    Vector3 readLinearAcceleration();    // In m/s²
    Vector3 readGravity();               // In m/s²

private:
    uint8_t i2c_address;

    void write8(uint8_t reg, uint8_t value);
    uint8_t read8(uint8_t reg);
    void readLen(uint8_t reg, uint8_t* buffer, uint8_t len);
    int16_t read16(uint8_t reg);
    float convertAngle(int16_t raw);     // Convert raw angle to degrees
    float convertVector(int16_t raw);    // Convert raw vector (accel/gyro) to units
    bool waitForCalibration();
};

#endif // BNO055_HPP

