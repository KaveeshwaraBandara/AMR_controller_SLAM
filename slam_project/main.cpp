#include "LidarReader.hpp"
#include "OccupancyGrid.hpp"
#include <iostream>

void printGrid(const OccupancyGrid& grid, int width, int height); // forward decl

int main() {
    LidarReader lidar("/dev/ttyUSB0", 1000000);
    if (!lidar.connect()) {
        std::cerr << "Failed to connect to LIDAR\n";
        return -1;
    }

    auto scan = lidar.getScan();
    std::cout << "Scan received with " << scan.size() << " points.\n";

    OccupancyGrid grid(100, 100, 0.05f); // 5cm resolution, 5x5m map
    grid.updateWithScan(scan);

    printGrid(grid, 100, 100);

    lidar.stop();
    return 0;
}

