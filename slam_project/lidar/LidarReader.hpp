#ifndef LIDAR_READER_HPP
#define LIDAR_READER_HPP

#include <vector>
#include <utility>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

class LidarReader {
public:
    LidarReader(const char* port, unsigned int baudrate);
    ~LidarReader();

    bool connect();
    std::vector<std::pair<float, float>> getScan();  // angle, distance (m)
    void stop();

private:
    const char* port_;
    unsigned int baudrate_;
    sl::ILidarDriver* drv_;
    sl::IChannel* channel_;
};

#endif

