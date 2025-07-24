#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/twist.pb.h>

#include "OccupancyGrid.hpp"
#include "ICPMatcher.hpp"
#include "VelocityEKF.hpp"
#include "PoseEKF.hpp"

#include <opencv2/opencv.hpp>
#include <mutex>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include <unistd.h>

#define LOOP_INTERVAL_MS 500

std::mutex scan_mutex;
gz::msgs::LaserScan latest_scan_msg;
bool scan_received = false;

std::mutex imu_mutex;
gz::msgs::IMU latest_imu_msg;
bool imu_received = false;

void OnScan(const gz::msgs::LaserScan &msg) {
    std::lock_guard<std::mutex> lock(scan_mutex);
    latest_scan_msg = msg;
    scan_received = true;
}

void OnIMU(const gz::msgs::IMU &msg) {
    std::lock_guard<std::mutex> lock(imu_mutex);
    latest_imu_msg = msg;
    imu_received = true;
}

std::vector<std::pair<float, float>> parseScan() {
    std::vector<std::pair<float, float>> scan;
    std::lock_guard<std::mutex> lock(scan_mutex);
    if (!scan_received) return scan;

    float angle = latest_scan_msg.angle_min();
    float step = latest_scan_msg.angle_step();

    for (int i = 0; i < latest_scan_msg.ranges_size(); ++i) {
        float dist = latest_scan_msg.ranges(i);
        if (dist < latest_scan_msg.range_min() || dist > latest_scan_msg.range_max())
            continue;
        scan.emplace_back(angle * 180.0f / M_PI, dist);
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

int main() {
    gz::transport::Node node;
    node.Subscribe("/world/world_demo/model/tugbot/link/scan_front/sensor/scan_front/scan", &OnScan);
    node.Subscribe("/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu", &OnIMU);

    VelocityEKF velocityEKF;
    PoseEKF poseEKF;
    OccupancyGrid grid(250, 250, 0.05f);
    std::vector<Pose2D> trajectory;
    std::vector<cv::Point2f> prev_cloud;

    auto last_time = std::chrono::steady_clock::now();
    //sendVelocity(node, 0.03f, 0.0f);

    float max_linear_velocity = 0.1f;
    float linear_vel = 0.0f;

    for (int frame = 0; frame < 100; ++frame) {
 

if (frame <= 30) {
    // Linear acceleration
    linear_vel = max_linear_velocity * (frame / 30.0f);
} else if (frame <= 70) {
    // Constant speed
    linear_vel = max_linear_velocity;
} else {
    // Linear deceleration
    linear_vel = max_linear_velocity * (1.0f - ((frame - 70.0f) / 30.0f));
}

// Send computed velocity
sendVelocity(node, linear_vel, 0.0f);

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        if (!imu_received || !scan_received) {
            std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_INTERVAL_MS));
            continue;
        }

        gz::msgs::IMU imu_msg;
        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            imu_msg = latest_imu_msg;
        }

        // Extract yaw and yaw rate from IMU quaternion and angular velocity
        auto q = imu_msg.orientation();
        float yaw_imu = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
                                   1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
        float yaw_rate_imu = imu_msg.angular_velocity().z();

        auto raw_scan = parseScan();
        auto current_cloud = toPointCloud(raw_scan);

        float v_icp = 0.0f;
        float w_icp = 0.0f;
        float dx = 0.0f, dy = 0.0f, dtheta = 0.0f;

        if (!prev_cloud.empty()) {
            cv::Mat Tr = runICP(prev_cloud, current_cloud);
            dx = Tr.at<double>(0, 2);
            dy = Tr.at<double>(1, 2);
            dtheta = std::atan2(Tr.at<double>(1, 0), Tr.at<double>(0, 0));

            v_icp = std::sqrt(dx*dx + dy*dy) / dt;
            w_icp = dtheta / dt;
        }

        // Fuse velocity: ICP linear velocity + IMU angular velocity
        VelocityEKF::Vector2f vel_meas;
        vel_meas << v_icp, yaw_rate_imu;

        velocityEKF.predict(dt);
        velocityEKF.correct(vel_meas);
        auto vel_est = velocityEKF.getState();

        // Pose EKF prediction using fused velocities
        poseEKF.predict(vel_est(0), vel_est(1), dt);

        // ICP pose measurement for correction
        PoseEKF::Vector3f pose_meas;
        if (!prev_cloud.empty()) {
            // Correct pose by ICP relative displacement from previous pose
            auto pose_state = poseEKF.getState();
            pose_meas << pose_state(0) + dx, pose_state(1) + dy, pose_state(2) + dtheta;
        } else {
            // First frame, no correction
            auto pose_state = poseEKF.getState();
            pose_meas = pose_state;
        }

        poseEKF.correct(pose_meas);

        // Save trajectory
        auto pose_state = poseEKF.getState();
        Pose2D current_pose = { pose_state(0), pose_state(1), pose_state(2), poseEKF.getCovariance() };
        trajectory.push_back(current_pose);

        // Update occupancy grid with transformed scan points
        auto global_points = transformToGlobal(raw_scan, current_pose);
        grid.updateWithGlobalPoints(global_points);

        prev_cloud = current_cloud;

        // Command constant velocity forward
        //sendVelocity(node, 0.03f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_INTERVAL_MS));
    }


    sendVelocity(node, 0.0f, 0.0f);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    grid.updateCostMap(0.3f);
    grid.saveAsImageWithTrajectory("final_map_with_trajectory_foward.png", trajectory);

    sleep(2);
    auto turn_start = std::chrono::steady_clock::now();
    gz::msgs::IMU imu_msg;
    auto q = imu_msg.orientation();
    float yaw_imu_start = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
                                   1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
    //serial.sendCommand(0, 0.3);
    sendVelocity(node, 0.0f, 0.3f);
    usleep(10472000);  // or wait for sensor feedback
    //serial.sendCommand(0, 0);
    sendVelocity(node, 0.0f, 0.0f);
    auto turn_end = std::chrono::steady_clock::now();
    q = imu_msg.orientation();
    float yaw_imu_end = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
                                   1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
    
    float dt_turn = std::chrono::duration<float>(turn_end - turn_start).count();
    poseEKF.predict(0.0f, (yaw_imu_start-yaw_imu_end)/dt_turn, dt_turn);
    sleep(2);
    auto pose_state =  poseEKF.getState();
    Pose2D current_pose = { pose_state(0), pose_state(1), pose_state(2), poseEKF.getCovariance() };
trajectory.push_back(current_pose);
    auto raw_scan = parseScan();
    auto global_points = transformToGlobal(raw_scan, current_pose);
    grid.updateWithGlobalPoints(global_points);
    grid.saveAsImageWithTrajectory("final_map_with_trajectory_foward.png", trajectory);
    std::vector<cv::Point2f> prev_cloud2;
    
prev_cloud2 = toPointCloud(raw_scan);

    max_linear_velocity = 0.1f;
    linear_vel = 0.0f;

    for (int frame = 0; frame < 100; ++frame) {
 

if (frame <= 30) {
    // Linear acceleration
    linear_vel = max_linear_velocity * (frame / 30.0f);
} else if (frame <= 70) {
    // Constant speed
    linear_vel = max_linear_velocity;
} else {
    // Linear deceleration
    linear_vel = max_linear_velocity * (1.0f - ((frame - 70.0f) / 30.0f));
}

// Send computed velocity
sendVelocity(node, linear_vel, 0.0f);

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        if (!imu_received || !scan_received) {
            std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_INTERVAL_MS));
            continue;
        }

        gz::msgs::IMU imu_msg;
        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            imu_msg = latest_imu_msg;
        }

        // Extract yaw and yaw rate from IMU quaternion and angular velocity
        auto q = imu_msg.orientation();
        float yaw_imu = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
                                   1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
        float yaw_rate_imu = imu_msg.angular_velocity().z();

        auto raw_scan = parseScan();
        auto current_cloud = toPointCloud(raw_scan);

        float v_icp = 0.0f;
        float w_icp = 0.0f;
        float dx = 0.0f, dy = 0.0f, dtheta = 0.0f;

        if (!prev_cloud2.empty()) {
            cv::Mat Tr = runICP(prev_cloud2, current_cloud);
            dx = Tr.at<double>(0, 2);
            dy = Tr.at<double>(1, 2);
            dtheta = std::atan2(Tr.at<double>(1, 0), Tr.at<double>(0, 0));

            v_icp = std::sqrt(dx*dx + dy*dy) / dt;
            w_icp = dtheta / dt;
        }

        // Fuse velocity: ICP linear velocity + IMU angular velocity
        VelocityEKF::Vector2f vel_meas;
        vel_meas << v_icp, yaw_rate_imu;

        velocityEKF.predict(dt);
        velocityEKF.correct(vel_meas);
        auto vel_est = velocityEKF.getState();

        // Pose EKF prediction using fused velocities
        poseEKF.predict(vel_est(0), vel_est(1), dt);

        // ICP pose measurement for correction
        PoseEKF::Vector3f pose_meas;
        if (!prev_cloud2.empty()&& frame> 0) {
        std::cout << "methentath awa bn\n";
            // Correct pose by ICP relative displacement from previous pose
            auto pose_state = poseEKF.getState();
            pose_meas << pose_state(0) + dx, pose_state(1) + dy, pose_state(2) + dtheta;
        } else {
            // First frame, no correction
            std::cout << "hallo machan mn methana\n";
            dx=dy=dtheta=0;		
          pose_meas = poseEKF.getState();
          continue;
        }

        poseEKF.correct(pose_meas);

        // Save trajectory
        auto pose_state = poseEKF.getState();
        Pose2D current_pose = { pose_state(0), pose_state(1), pose_state(2), poseEKF.getCovariance() };
        trajectory.push_back(current_pose);

        // Update occupancy grid with transformed scan points
        auto global_points = transformToGlobal(raw_scan, current_pose);
        grid.updateWithGlobalPoints(global_points);

        prev_cloud2 = current_cloud;

        // Command constant velocity forward
        //sendVelocity(node, 0.03f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_INTERVAL_MS));
    }


    sendVelocity(node, 0.0f, 0.0f);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    grid.updateCostMap(0.3f);
    grid.saveAsImageWithTrajectory("final_map_with_trajectory_back and full.png", trajectory);

    std::cout << "Simulation complete. Map saved to final_map_with_trajectory.png\n";

    return 0;
}
