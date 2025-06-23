#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/laserscan.pb.h>

#include "OccupancyGrid.hpp"
#include "ICPMatcher.hpp"
#include "EKF.hpp"
#include "Astar.hpp"

#include <opencv2/opencv.hpp>
#include <mutex>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

#define LOOP_INTERVAL_MS 500

// ---------- Shared Data for Lidar ----------
std::mutex scan_mutex;
gz::msgs::LaserScan latest_scan_msg;
bool scan_received = false;

void OnScan(const gz::msgs::LaserScan &msg) {
    std::lock_guard<std::mutex> lock(scan_mutex);
    latest_scan_msg = msg;
    scan_received = true;
}

std::vector<std::pair<float, float>> parseScan() {
    std::vector<std::pair<float, float>> scan;
    std::lock_guard<std::mutex> lock(scan_mutex);

    if (!scan_received) return scan;

    float angle = latest_scan_msg.angle_min();
    float step = latest_scan_msg.angle_step();

    for (int i = 0; i < latest_scan_msg.ranges_size(); ++i) {
        float dist = latest_scan_msg.ranges(i);
        float angle_deg = angle * 180.0f / M_PI;
        scan.emplace_back(angle_deg, dist);
        angle += step;
    }
    return scan;
}

// ---------- Convert Scan to Point Cloud ----------
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

// ---------- Send Velocity Commands ----------
void sendVelocity(gz::transport::Node &node, float linear, float angular) {
    static auto velPub = node.Advertise<gz::msgs::Twist>("/model/tugbot/cmd_vel");
    gz::msgs::Twist msg;
    msg.mutable_linear()->set_x(linear);
    msg.mutable_angular()->set_z(angular);
    velPub.Publish(msg);
}

int main() {
    // Gazebo Transport node
    gz::transport::Node node;
    node.Subscribe("/world/world_demo/model/tugbot/link/scan_front/sensor/scan_front/scan", &OnScan);

    EKF ekf;
    OccupancyGrid grid(250, 250, 0.05f);
    std::vector<Pose2D> trajectory;
    std::vector<cv::Point2f> prev_cloud;

    auto last_time = std::chrono::steady_clock::now();

    // Initial motion
    sendVelocity(node, 0.03, 0.0f);

    for (int frame = 0; frame < 100; ++frame) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        float v = 0.03f, w = 0.0f;
        ekf.predict(v, w, dt);

        auto raw_scan = parseScan();
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

        prev_cloud = current_cloud;
        sendVelocity(node, 0.03f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_INTERVAL_MS));
    }

    // Stop and turn
    sendVelocity(node, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    auto turn_start = std::chrono::steady_clock::now();
    sendVelocity(node, 0.0, 0.3);
    std::this_thread::sleep_for(std::chrono::milliseconds(10472));
    sendVelocity(node, 0.0, 0.0);
    auto turn_end = std::chrono::steady_clock::now();
    ekf.predict(0.0f, 0.3f, std::chrono::duration<float>(turn_end - turn_start).count());

    // Final stop
    sendVelocity(node, 0.0, 0.0);
    grid.updateCostMap(0.3f);
    grid.saveAsImageWithTrajectory("final_map_with_trajectory.png", trajectory);

    std::cout << "Simulation complete. Map saved.\n";
    return 0;
}
