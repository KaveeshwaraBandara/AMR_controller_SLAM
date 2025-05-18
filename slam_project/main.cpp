#include "LidarReader.hpp"
#include "OccupancyGrid.hpp"
#include "ICPMatcher.hpp"
#include <iostream>

void printGrid(const OccupancyGrid& grid, int width, int height); // forward decl


int main() {
    LidarReader lidar("/dev/ttyUSB0", 1000000);
    if (!lidar.connect()) {
        std::cerr << "Failed to connect to LIDAR\n";
        return -1;
    }
    
    std::vector<cv::Point2f> trajectory;


    //auto scan = lidar.getScan();
    //std::cout << "Scan received with " << scan.size() << " points.\n";

    OccupancyGrid grid(500, 500, 0.05f); // 5cm resolution, 5x5m map
    //grid.updateWithScan(scan);
    //grid.saveAsImage("map.png");

    
    
    
    cv::Mat globalPose = cv::Mat::eye(3, 3, CV_64F);
std::vector<cv::Point2f> prevScan;
int scanCount = 0;

while (scanCount<20) {
  scanCount++;
    auto scan = lidar.getScan();
    auto currScan = convertToPoint2f(scan);

    if (prevScan.empty()) {
        prevScan = currScan;
        auto transformed = currScan;  // First scan, no transform
        grid.updateWithGlobalPoints(transformed);
        continue;
    }

    cv::Mat Tr = ICPMatcher::align(prevScan, currScan);
    globalPose = globalPose * Tr;
    float x = globalPose.at<double>(0, 2);
    float y = globalPose.at<double>(1, 2);
    trajectory.emplace_back(x, y);


    std::vector<cv::Point2f> transformedScan;
    cv::transform(currScan, transformedScan, globalPose);
    grid.updateWithGlobalPoints(transformedScan);

    prevScan = currScan;

    // (Optional) visualize or save
    //grid.saveAsImage("map.png");
    grid.saveAsImageWithPath("map_with_trajectory.png", trajectory);
}
lidar.stop();

    
    
    
    return 0;
}

