#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "Odometry.hpp"
#include <sstream>

struct LidarPoint {
    float angle_deg;
    float distance_m;
};

std::vector<LidarPoint> readLidarCSV(const std::string& filename) {
    std::vector<LidarPoint> points;
    std::ifstream file(filename);
    std::string line;
    bool first_line = true;
    while (std::getline(file, line)) {
        if (first_line) { 
            first_line = false;
            continue;
        }
        std::stringstream ss(line);
        std::string angle_str, dist_str;
        if (std::getline(ss, angle_str, ',') && std::getline(ss, dist_str)) {
            LidarPoint p;
            p.angle_deg = std::stof(angle_str);
            p.distance_m = std::stof(dist_str);
            points.push_back(p);
        }
    }
    return points;
}

int main() {
    const int map_size = 500;
    const float meters_per_pixel = 0.02f;
    const float wheel_base = 0.2f;

    cv::Mat map(map_size, map_size, CV_8UC3, cv::Scalar(128, 128, 128)); 
    Pose robotPose = {map_size / 2.0f, map_size / 2.0f, 0.0f};

    const int total_steps = 3; // Number of CSV files you have: scan1.csv, scan2.csv, ...

    for (int step = 1; step <= total_steps; ++step) {
        // ---- Load current scan ----
        std::string filename = "../scan" + std::to_string(step) + ".csv";
        std::vector<LidarPoint> scan = readLidarCSV(filename);
        std::cout << "Loaded " << filename << " with " << scan.size() << " points.\n";

        // ---- Odometry Update ----
        float delta_left = 0.04f;
        float delta_right = 0.05f;
        float imu_angular_velocity = 0.05f;
        float dt = 0.1f;

        robotPose = updateOdometry(robotPose, delta_left, delta_right, wheel_base);
        robotPose = updateWithIMU(robotPose, imu_angular_velocity, dt);

        // ---- Map Update ----
        for (const auto& p : scan) {
            if (p.distance_m > 0.1f && p.distance_m < 6.0f) {
                float angle_rad = (p.angle_deg * M_PI / 180.0f) + robotPose.theta;
                float local_x = p.distance_m * cos(angle_rad);
                float local_y = p.distance_m * sin(angle_rad);

                int map_x = static_cast<int>((robotPose.x + local_x / meters_per_pixel));
                int map_y = static_cast<int>((robotPose.y + local_y / meters_per_pixel));

                if (map_x >= 0 && map_x < map_size && map_y >= 0 && map_y < map_size) {
                    map.at<cv::Vec3b>(map_y, map_x) = cv::Vec3b(0, 0, 0);
                }
            }
        }

        // ---- Draw Robot ----
        int robot_pixel_x = static_cast<int>(robotPose.x);
        int robot_pixel_y = static_cast<int>(robotPose.y);
        if (robot_pixel_x >= 0 && robot_pixel_x < map_size && robot_pixel_y >= 0 && robot_pixel_y < map_size) {
            cv::circle(map, cv::Point(robot_pixel_x, robot_pixel_y), 3, cv::Scalar(0, 255, 0), -1);
        }

        // ---- Show Map ----
        cv::imshow("SLAM Map", map);
        cv::waitKey(500); // Pause 0.5 seconds between steps
    }

    // ---- Save final Map ----
    cv::imwrite("slam_output.png", map);
    std::cout << "SLAM map saved as slam_output.png!" << std::endl;

    return 0;
}

