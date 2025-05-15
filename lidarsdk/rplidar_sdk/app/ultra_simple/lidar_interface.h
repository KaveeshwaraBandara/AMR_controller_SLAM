#ifndef LIDAR_INTERFACE_H
#define LIDAR_INTERFACE_H

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace slamtec {
    bool initializeLidar(int argc, const char* argv[]);
    void readLidarData();
    void cleanupLidar();
}

#endif

