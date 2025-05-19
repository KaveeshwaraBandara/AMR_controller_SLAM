#include "LidarReader.hpp"
#include "OccupancyGrid.hpp"
#include "ICPMatcher.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>



//struct Pose2D {
  //  float x = 0.0f, y = 0.0f, theta = 0.0f;
//};

// Convert polar scan to 2D point cloud (local)
std::vector<cv::Point2f> toPointCloud(const std::vector<std::pair<float, float>>& scan) {
    std::vector<cv::Point2f> cloud;
    for (const auto& [angle_deg, dist_m] : scan) {
        float theta = angle_deg * CV_PI / 180.0f;
        float x = dist_m * std::cos(theta);
        float y = dist_m * std::sin(theta);
        cloud.emplace_back(x, y);
    }
    return cloud;
}

// Transform local scan using robot pose
std::vector<std::pair<float, float>> transformToGlobal(const std::vector<std::pair<float, float>>& scan, const Pose2D& pose) {
    std::vector<std::pair<float, float>> transformed;
    float c = std::cos(pose.theta);
    float s = std::sin(pose.theta);

    for (const auto& [angle_deg, dist_m] : scan) {
        float angle_rad = angle_deg * CV_PI / 180.0f;
        float x_local = dist_m * std::cos(angle_rad);
        float y_local = dist_m * std::sin(angle_rad);

        float x_global = x_local * c - y_local * s + pose.x;
        float y_global = x_local * s + y_local * c + pose.y;

        transformed.emplace_back(x_global, y_global);
    }
    return transformed;
}

int main() {
    LidarReader lidar("/dev/ttyUSB0", 1000000);
    if (!lidar.connect()) {
        std::cerr << "Failed to connect to LIDAR\n";
        return -1;
    }
    
    
    //std::vector<std::pair<float, float>> trajectory;
    std::vector<Pose2D> trajectory;

    OccupancyGrid grid(500, 500, 0.05f);  // 25x25 meter map
    Pose2D robot_pose;
    std::vector<cv::Point2f> prev_cloud;

    for (int frame = 0; frame < 20; ++frame) {
        auto raw_scan = lidar.getScan();
        auto current_cloud = toPointCloud(raw_scan);

        if (!prev_cloud.empty()) {
            cv::Mat Tr = runICP(prev_cloud, current_cloud);
            float dx = Tr.at<double>(0, 2);
            float dy = Tr.at<double>(1, 2);
            float dtheta = std::atan2(Tr.at<double>(1, 0), Tr.at<double>(0, 0));

            // Update robot pose using relative motion
            float c = std::cos(robot_pose.theta);
            float s = std::sin(robot_pose.theta);
            robot_pose.x += dx * c - dy * s;
            robot_pose.y += dx * s + dy * c;
            robot_pose.theta += dtheta;
            //trajectory.emplace_back(robot_pose.x, robot_pose.y);
            trajectory.emplace_back(robot_pose);

        }

        // Convert current scan to global and update map
        auto transformed = transformToGlobal(raw_scan, robot_pose);
        grid.updateWithGlobalPoints(transformed);
        grid.showLiveMap(trajectory);


        prev_cloud = current_cloud;
        std::cout << "Frame " << frame << ": pose=(" << robot_pose.x << ", " << robot_pose.y << ", " << robot_pose.theta << ")\n";
    }

    lidar.stop();
    grid.saveAsImageWithTrajectory("map_with_trajectory.png", trajectory);

//    grid.saveAsImage("map.png");
    std::cout << "Saved map to map.png\n";

    return 0;
}

