#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/imu.pb.h>

#include "OccupancyGrid.hpp"
#include "ICPMatcher.hpp"
#include "EKF.hpp"
#include "IMUReader.hpp"

#include <opencv2/opencv.hpp>
#include <mutex>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

// ---------- Shared Data for Sensors ----------
std::mutex scan_mutex;
gz::msgs::LaserScan latest_scan_msg;
bool scan_received = false;

std::mutex imu_mutex;
IMU imu_sensor;
bool imu_received = false;

// ---------- Sensor Callbacks ----------
void OnScan(const gz::msgs::LaserScan &msg) {
    std::lock_guard<std::mutex> lock(scan_mutex);
    latest_scan_msg = msg;
    scan_received = true;
}

void OnIMU(const gz::msgs::IMU &msg) {
    IMU::Data data;
    data.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    data.linear_acceleration << 
        msg.linear_acceleration().x(),
        msg.linear_acceleration().y(),
        msg.linear_acceleration().z();
    
    data.angular_velocity << 
        msg.angular_velocity().x(),
        msg.angular_velocity().y(),
        msg.angular_velocity().z();
    
    const auto& q = msg.orientation();
    data.orientation(2) = atan2(2.0f*(q.w()*q.z() + q.x()*q.y()), 
                          1.0f - 2.0f*(q.y()*q.y() + q.z()*q.z())); // yaw
    
    {
        std::lock_guard<std::mutex> lock(imu_mutex);
        imu_sensor.update(data);
        imu_received = true;
    }
}

// ---------- Utility Functions ----------
std::vector<std::pair<float, float>> parseScan() {
    std::vector<std::pair<float, float>> scan;
    std::lock_guard<std::mutex> lock(scan_mutex);

    if (!scan_received) return scan;

    float angle = latest_scan_msg.angle_min();
    float step = latest_scan_msg.angle_step();

    for (int i = 0; i < latest_scan_msg.ranges_size(); ++i) {
        float dist = latest_scan_msg.ranges(i);
        if (!std::isfinite(dist)) continue;
        float angle_deg = angle * 180.0f / M_PI;
        scan.emplace_back(angle_deg, dist);
        angle += step;
    }
    return scan;
}

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

void sendVelocity(gz::transport::Node &node, float linear, float angular) {
    static auto velPub = node.Advertise<gz::msgs::Twist>("/model/tugbot/cmd_vel");
    gz::msgs::Twist msg;
    msg.mutable_linear()->set_x(linear);
    msg.mutable_angular()->set_z(angular);
    velPub.Publish(msg);
}

// ---------- Movement Functions ----------
void moveForward(gz::transport::Node &node, EKF &ekf, OccupancyGrid &grid, 
                std::vector<Pose2D> &trajectory, float duration_sec, 
                float speed = 0.08f, bool record_scan = true) {
    auto start_time = std::chrono::steady_clock::now();
    auto last_time = start_time;
    std::vector<cv::Point2f> prev_cloud;
    
    sendVelocity(node, speed, 0.0f);
    
    while (true) {
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - start_time).count();
        if (elapsed >= duration_sec) break;
        
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        // Get IMU data for prediction
        Eigen::Vector3f imu_gyro = imu_sensor.getAngularVelocity();
        ekf.predict(speed, 0.0f, dt, Eigen::Vector3f::Zero(), imu_gyro);

        if (record_scan) {
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

                ekf.correctLidar(z);
            }

            // Update map only when recording scans
            Eigen::Vector3f x = ekf.getState();
            Pose2D pose = {x(0), x(1), x(2)};
            trajectory.push_back(pose);
            
            auto global_points = transformToGlobal(raw_scan, pose);
            grid.updateWithGlobalPoints(global_points);
            
            prev_cloud = toPointCloud(raw_scan);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    sendVelocity(node, 0.0f, 0.0f);
}

void rotateInPlace(gz::transport::Node &node, EKF &ekf, float angle_deg, float speed = 0.3f) {
    float target_angle = ekf.getState()(2) + angle_deg * M_PI / 180.0f;
    auto start_time = std::chrono::steady_clock::now();
    auto last_time = start_time;
    
    // Determine rotation direction
    float angular_vel = (angle_deg > 0) ? speed : -speed;
    sendVelocity(node, 0.0f, angular_vel);
    
    while (true) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        // Use IMU for rotation estimation
        Eigen::Vector3f imu_gyro = imu_sensor.getAngularVelocity();
        ekf.predict(0.0f, angular_vel, dt, Eigen::Vector3f::Zero(), imu_gyro);
        
        // Check if we've reached target angle
        float current_angle = ekf.getState()(2);
        if (fabs(current_angle - target_angle) < 0.05f) break;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    sendVelocity(node, 0.0f, 0.0f);
}

// ---------- Main SLAM Routine ----------
int main() {
    gz::transport::Node node;
    node.Subscribe("/world/world_demo/model/tugbot/link/scan_front/sensor/scan_front/scan", &OnScan);
    node.Subscribe("/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu", &OnIMU);

    // Wait for sensors
    std::cout << "Waiting for sensors..." << std::endl;
    while (!scan_received || !imu_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Sensors ready!" << std::endl;

    // Initialize SLAM components
    EKF ekf;
    OccupancyGrid grid(500, 500, 0.05f);
    std::vector<Pose2D> trajectory;

    // Calibrate IMU (values should be measured from actual IMU when stationary)
    ekf.calibrateIMU(Eigen::Vector3f(0.01f, -0.02f, 0.05f), 
                    Eigen::Vector3f(0.001f, 0.002f, -0.001f));

    // ---------- Movement Pattern ----------
    
      // 2. Rotate 180° (no scanning)
    std::cout << "Rotating 180° (no scanning)" << std::endl;
    rotateInPlace(node, ekf, 180.0f);
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Pause after rotation
    // 1. Move forward while scanning
    std::cout << "Moving forward (scanning)" << std::endl;
    moveForward(node, ekf, grid, trajectory, 10.0f, 0.08f, true);
    
    // 2. Rotate 180° (no scanning)
    std::cout << "Rotating 180° (no scanning)" << std::endl;
    rotateInPlace(node, ekf, 180.0f);
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Pause after rotation
    
    // 3. Move back to start while scanning
    std::cout << "Moving back (scanning)" << std::endl;
    moveForward(node, ekf, grid, trajectory, 10.0f, 0.08f, true);
    
    // ---------- Finalization ----------
    sendVelocity(node, 0.0f, 0.0f);
    grid.updateCostMap(0.3f);
    grid.saveAsImageWithTrajectory("final_map_forward_back.png", trajectory);

    std::cout << "SLAM complete. Map saved." << std::endl;
    return 0;
}
