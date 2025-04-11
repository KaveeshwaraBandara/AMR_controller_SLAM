#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include <map>

#define MAP_SIZE 800
#define SCALE 100.0f  // 1 meter = 100 pixels

struct LidarReading {
    int frame_id;
    float angle_deg;
    float distance_m;
};

std::vector<LidarReading> loadLidarCSV(const std::string& filename) {
    std::vector<LidarReading> data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file.\n";
        return data;
    }

    std::string line;
    std::getline(file, line); // skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        LidarReading reading;
        std::getline(ss, token, ',');
        reading.frame_id = std::stoi(token);
        std::getline(ss, token, ',');
        reading.angle_deg = std::stof(token);
        std::getline(ss, token, ',');
        reading.distance_m = std::stof(token);
        data.push_back(reading);
    }
    return data;
}

void drawLine(cv::Mat& map, cv::Point start, cv::Point end, uchar value) {
    cv::LineIterator it(map, start, end, 8);
    for (int i = 0; i < it.count; i++, ++it) {
        if (it.pos().x >= 0 && it.pos().x < map.cols && it.pos().y >= 0 && it.pos().y < map.rows) {
            map.at<uchar>(it.pos()) = value; // mark free space (e.g., 128)
        }
    }
}

int main() {
    auto data = loadLidarCSV("../simulated_lidar.csv");
    std::cout << "Total readings loaded: " << data.size() << std::endl;

    // Group by frame ID
    std::map<int, std::vector<LidarReading>> frames;
    for (const auto& reading : data) {
        frames[reading.frame_id].push_back(reading);
    }

    std::cout << "Total frames found: " << frames.size() << std::endl;

    for (const auto& [frame_id, readings] : frames) {
        std::cout << "Processing frame: " << frame_id << std::endl;

        // Create a grayscale occupancy grid
        cv::Mat map(MAP_SIZE, MAP_SIZE, CV_8UC1, cv::Scalar(128));  // 128 = unknown

        cv::Point robot_pos(MAP_SIZE / 2, MAP_SIZE / 2); // robot at center

        for (const auto& p : readings) {
            float rad = p.angle_deg * M_PI / 180.0f;
            float x = p.distance_m * cos(rad);
            float y = p.distance_m * sin(rad);

            cv::Point end = robot_pos + cv::Point(x * SCALE, y * SCALE);
            drawLine(map, robot_pos, end, 200);  // Free space: brighter
            if (end.x >= 0 && end.x < map.cols && end.y >= 0 && end.y < map.rows) {
                map.at<uchar>(end) = 0; // Mark obstacle (black)
            }
        }

        // Display
        cv::imshow("Occupancy Grid Map", map);
        std::string filename = "frame_" + std::to_string(frame_id) + ".png";
        cv::imwrite(filename, map);
        cv::waitKey(100); // display for 100 ms
    }

    std::cout << "All frames processed." << std::endl;
    cv::waitKey(0); // Keep the last window open
    return 0;
}

