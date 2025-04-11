#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

// Constants
const int MAP_SIZE = 800;
const float MAP_RESOLUTION = 0.05f; // meters per cell
const cv::Point2f MAP_ORIGIN(MAP_SIZE / 2, MAP_SIZE / 2);

// Lidar point struct
struct LidarPoint {
    float angle_deg;
    float distance_m;
};

// Load LiDAR data
std::vector<LidarPoint> loadCSV(const std::string& filename) {
    std::vector<LidarPoint> data;
    std::ifstream file(filename);
    std::string line;

    std::getline(file, line); // Skip header if exists
    if (line.find("angle") == std::string::npos) {
        std::istringstream ss(line);
        std::string a, d;
        std::getline(ss, a, ',');
        std::getline(ss, d);
        data.push_back({std::stof(a), std::stof(d)});
    }

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string a, d;
        if (std::getline(ss, a, ',') && std::getline(ss, d)) {
            data.push_back({std::stof(a), std::stof(d)});
        }
    }
    return data;
}

// Draw line using Bresenham to mark free space
void bresenhamLine(cv::Mat& map, cv::Point start, cv::Point end, uchar value) {
    int dx = std::abs(end.x - start.x);
    int dy = std::abs(end.y - start.y);
    int sx = start.x < end.x ? 1 : -1;
    int sy = start.y < end.y ? 1 : -1;
    int err = dx - dy;

    while (start != end) {
        map.at<uchar>(start.y, start.x) = value; // Free space
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; start.x += sx; }
        if (e2 < dx)  { err += dx; start.y += sy; }
    }
}

// World (meters) → map (pixels)
cv::Point2i worldToMap(float x, float y) {
    int mx = static_cast<int>(x / MAP_RESOLUTION + MAP_ORIGIN.x);
    int my = static_cast<int>(-y / MAP_RESOLUTION + MAP_ORIGIN.y); // flip y
    return {mx, my};
}

int main() {
    std::string filename = "../lidar_data.csv";
    std::vector<LidarPoint> scan = loadCSV(filename);

    cv::Mat map(MAP_SIZE, MAP_SIZE, CV_8UC1, cv::Scalar(127)); // 127 = unknown

    for (const auto& p : scan) {
        float angle_rad = p.angle_deg * M_PI / 180.0f;
        float x = p.distance_m * std::cos(angle_rad);
        float y = p.distance_m * std::sin(angle_rad);

        cv::Point2i start = worldToMap(0.0f, 0.0f);       // Robot center
        cv::Point2i end   = worldToMap(x, y);             // Hit point

        // Draw free space
        bresenhamLine(map, start, end, 255); // free = white
        // Mark end point as occupied
        if (end.x >= 0 && end.x < MAP_SIZE && end.y >= 0 && end.y < MAP_SIZE)
            map.at<uchar>(end.y, end.x) = 0; // occupied = black
    }

    // Show the occupancy grid
    cv::imshow("Occupancy Grid", map);
    cv::waitKey(0);
    return 0;
}

