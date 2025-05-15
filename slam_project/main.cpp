#include "LidarReader.hpp"
#include <iostream>

int main() {
    LidarReader lidar("/dev/ttyUSB0", 1000000); // S2L default
    if (!lidar.connect()) {
        std::cerr << "Failed to connect to LIDAR\n";
        return -1;
    }

    auto scan = lidar.getScan();
    std::cout << "Scan received with " << scan.size() << " points.\n";

    // Further: update occupancy grid here

    lidar.stop();
    return 0;
}

