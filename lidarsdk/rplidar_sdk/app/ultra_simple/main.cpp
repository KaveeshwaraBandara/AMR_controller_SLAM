#include "lidar_interface.h"
#include <iostream>

int main(int argc, const char* argv[]) {
    if (!slamtec::initializeLidar(argc, argv)) {
        std::cerr << "Lidar initialization failed.\n";
        return -1;
    }

    while (true) {
        slamtec::readLidarData();
    }

    slamtec::cleanupLidar();
    return 0;
}

