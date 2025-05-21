#include "LidarReader.hpp"
#include "OccupancyGrid.hpp"
#include "ICPMatcher.hpp"
#include "EKF.hpp"
#include "IMUReader.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include "send.hpp"

//struct Pose2D {
//    float x = 0, y = 0, theta = 0;
//};

std::vector<cv::Point2f> toPointCloud(const std::vector<std::pair<float, float>>& scan) {
    std::vector<cv::Point2f> cloud;
    for (auto [angle_deg, dist_m] : scan) {
        float theta = angle_deg * CV_PI / 180.0f;
        cloud.emplace_back(dist_m * cos(theta), dist_m * sin(theta));
    }
    return cloud;
}

std::vector<std::pair<float, float>> transformToGlobal(const std::vector<std::pair<float, float>>& scan, const Pose2D& pose) {
    std::vector<std::pair<float, float>> global;
    float c = cos(pose.theta), s = sin(pose.theta);
    for (auto [angle_deg, dist_m] : scan) {
        float theta = angle_deg * CV_PI / 180.0f;
        float x = dist_m * cos(theta), y = dist_m * sin(theta);
        float gx = x * c - y * s + pose.x;
        float gy = x * s + y * c + pose.y;
        global.emplace_back(gx, gy);
    }
    return global;
}

int main() {
    LidarReader lidar("/dev/ttyUSB0", 1000000);
    if (!lidar.connect()) {
        std::cerr << "Lidar connect failed\n";
        return -1;
    }
    const char* portName = "/dev/ttyACM0"; // Adjust based on your device
    int baudRate = B9600; // Match with microcontroller's baud rate

    SerialPort serial(portName, baudRate);

    IMUReader imu;
    imu.initialize();

    EKF ekf;
    OccupancyGrid grid(250, 250, 0.05f);
    std::vector<Pose2D> trajectory;
    std::vector<cv::Point2f> prev_cloud;

    auto last_time = std::chrono::steady_clock::now();
    std::string message = "set 0.1 0";
    serial.sendData(message);

    for (int frame = 0; frame < 100; ++frame) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        imu.update();
        float v = imu.getLinearVelocity();
        float w = imu.getAngularVelocity();

        //ekf.predict(v, w, dt);

        auto raw_scan = lidar.getScan();
        ekf.predict(v, w, dt);
        auto current_cloud = toPointCloud(raw_scan);

        if (!prev_cloud.empty()) {
            cv::Mat Tr = runICP(prev_cloud, current_cloud);

            float dx = Tr.at<double>(0, 2);
            float dy = Tr.at<double>(1, 2);
            float dtheta = atan2(Tr.at<double>(1, 0), Tr.at<double>(0, 0));

            Eigen::Vector3f z;
            z << ekf.getState()(0) + dx,
                 ekf.getState()(1) + dy,
                 ekf.getState()(2) + dtheta;

            ekf.correct(z);
            Eigen::Vector3f x = ekf.getState();
            Eigen::Matrix3f P = ekf.getCovariance();
            trajectory.push_back({ x(0), x(1), x(2), P });

        }

        Eigen::Vector3f x = ekf.getState();
        Pose2D pose = { x(0), x(1), x(2) };
        trajectory.push_back(pose);

        auto global_points = transformToGlobal(raw_scan, pose);
        grid.updateWithGlobalPoints(global_points);
        //grid.showLiveMap(trajectory);  // OpenCV visualization

        prev_cloud = current_cloud;
        message = "set 0.1 0";
	serial.sendData(message);

        //if (cv::waitKey(10) == 'q') break;
    }

    message = "set 0 0";
    serial.sendData(message);
    lidar.stop();
    //update cost map
    float robot_radius = 0.3f;
    grid.updateCostMap(robot_radius);
    // Create cost map visualization
    cv::Mat cost_map_img(grid.getHeight(), grid.getWidth(), CV_8UC3, cv::Scalar(255, 255, 255));
    
    // Draw cost map
    for (int y = 0; y < grid.getHeight(); ++y) {
        for (int x = 0; x < grid.getWidth(); ++x) {
            float cost = grid.getCost(x, y);
            if (std::isinf(cost)) {
                cost_map_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Red for obstacles
            }
        }
    }
    grid.saveAsImageWithTrajectory("map_with_ellipse.png", trajectory);
    //grid.saveAsImageWithTrajectory("map_with_heading.png", trajectory);
    //cv::destroyAllWindows();
    return 0;
}

