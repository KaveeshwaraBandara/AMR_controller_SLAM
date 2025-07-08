#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

struct Point {
    float angle_deg;
    float distance_m;
};

// Read CSV: angle (degrees), distance (meters)
std::vector<Point> readCSV(const std::string& filename) {
    std::vector<Point> points;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        float angle, dist;
        if (sscanf(line.c_str(), "%f,%f", &angle, &dist) == 2) {
            if (dist > 0 && dist < 10) { // sanity check
                points.push_back({angle, dist});
            }
        }
    }
    return points;
}

int main() {
    std::string filename = "../lidar_data.csv";  // ../ because we're running from build/
    auto points = readCSV(filename);

    int imgSize = 800;
    cv::Mat map(imgSize, imgSize, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Point center(imgSize/2, imgSize/2);

    // Plot every point
    for (const auto& p : points) {
        float rad = p.angle_deg * M_PI / 180.0f;
        float x = p.distance_m * cos(rad);
        float y = p.distance_m * sin(rad);

        int px = static_cast<int>(center.x + x * 100);  // 1 meter = 100 pixels
        int py = static_cast<int>(center.y - y * 100);  // invert y

        if (px >= 0 && px < imgSize && py >= 0 && py < imgSize)
            cv::circle(map, {px, py}, 3, cv::Scalar(0, 0, 255), -1);  // red dot
    }

    // Draw robot center
    cv::circle(map, center, 5, cv::Scalar(0, 255, 0), -1);  // green

    cv::imshow("LiDAR View", map);
    cv::waitKey(0);
    return 0;
}

