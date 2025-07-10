#include "navigation.hpp"
#include <cmath>
#include <unistd.h>
#include <iostream>
#include "SLAMUtils.hpp"

Navigator::Navigator(OccupancyGrid& grid,
    AStarPlanner& planner,
    PoseEKF& poseEKF,
    VelocityEKF& velocityEKF,
    BNO055& imu,
    LidarReader& lidar,
    std::vector<Pose2D>& trajectory,
    std::vector<cv::Point2f>& prev_cloud,
    SerialPort& serial)
: grid_(grid), planner_(planner), poseEKF_(poseEKF),
velocityEKF_(velocityEKF), imu_(imu), lidar_(lidar),
trajectory_(trajectory), prev_cloud_(prev_cloud),
serial_(serial) {}


void Navigator::setGoal(int gx, int gy) {
    goal_x_ = static_cast<int>(gx / grid_.getResolution()) + grid_.getOriginX();
    goal_y_ = static_cast<int>(gy / grid_.getResolution()) + grid_.getOriginY();
//goal_x_ = gx;    
//goal_y_ = gy;
    
goal_set_ = true;
}

std::vector<std::pair<float, float>> Navigator::convertPathToWorld(const std::vector<std::pair<int, int>>& path) {
    std::vector<std::pair<float, float>> world_path;
    for (auto [x, y] : path) {
        float wx = (x - grid_.getOriginX()) * grid_.getResolution();
        float wy = (y - grid_.getOriginY()) * grid_.getResolution();
        world_path.emplace_back(wx, wy);
    }
    return world_path;
}

void Navigator::computeAndSendCommand(float curr_x, float curr_y, float curr_theta,
                                      float target_x, float target_y) {
    float dx = target_x - curr_x;
    float dy = target_y - curr_y;
    float dist = std::sqrt(dx * dx + dy * dy);
    float target_heading = std::atan2(dy, dx);
    float angle_diff = target_heading - curr_theta;

    // Normalize angle to [-π, π]
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    float v = std::min(0.05f, dist);  // Limit forward speed
    float w = angle_diff;

    // Clamp angular speed
    if (w > 1.0f) w = 1.0f;
    if (w < -1.0f) w = -1.0f;

    serial_.sendCommand(v, w);
}

void Navigator::navigateToGoal() {
    if (!goal_set_) return;

    std::cout << "[Navigator] Starting navigation...\n";
prev_cloud_.clear();

std::cout << "[SLAM] prev_cloud size at entry: " << prev_cloud_.size() << "\n";


    for (int step = 0; step < 50; ++step) {  // limit for demo
        // Get current time
        static auto last_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        // Read IMU data
        EulerAngles orientation = imu_.readEulerAngles();
        Vector3 angularVel = imu_.readAngularVelocity();

//float yaw_imu = orientaion.yaw*M_PI/180.0f;
//float yaw_rate_imu = angularVel.z*M_PI/180.0f;

        // Get Lidar scan
        auto scan = lidar_.getScan();
	//auto current_cloud = toPointCloud(scan);
        // Update pose using SLAM
        updateSLAM(scan, orientation, angularVel,
                   prev_cloud_, velocityEKF_, poseEKF_, trajectory_, grid_, dt,current_pose,statee);

        // Get current pose
        //auto pose = poseEKF_.getState();
        float x = statee(0), y = statee(1), theta = statee(2);

        // Compute path to goal
        int start_x = static_cast<int>(x / grid_.getResolution()) + grid_.getOriginX();
        int start_y = static_cast<int>(y / grid_.getResolution()) + grid_.getOriginY();
        //int goal_x = static_cast<int>(goal_x_ / grid_.getResolution()) + grid_.getOriginX();
        //int goal_y = static_cast<int>(goal_y_ / grid_.getResolution()) + grid_.getOriginY();

        auto path = planner_.plan(start_x, start_y, goal_x_, goal_y_);
std::cout << "[Navigator] start inside"<<start_x<<", "<<start_y<<"\n";	
std::cout << "[Navigator] goal inside"<<goal_x_<<", "<<goal_y_<<"\n";
	std::cout << "[Navigator] goal cost"<<grid_.getCost(goal_x_,goal_y_)<<"\n";
std::cout << "[Navigator] start cost"<<grid_.getCost(start_x,start_y)<<"\n";
        if (path.empty()) {
            std::cout << "[Navigator] No path found\n";
            serial_.sendCommand(0, 0);
            break;
        }

        // Determine next move
        if (path.size() < 2) {
            std::cout << "[Navigator] Goal reached\n";
            serial_.sendCommand(0, 0);
            break;
        }
//std::cout << "HELLO\n";
        auto [next_x, next_y] = path[1];
        float target_world_x = (next_x - grid_.getOriginX()) * grid_.getResolution();
        float target_world_y = (next_y - grid_.getOriginY()) * grid_.getResolution();
//std::cout << "SAHAN\n";
        float dx = target_world_x - x;
        float dy = target_world_y - y;
        float desired_theta = atan2(dy, dx);
        float angle_error = desired_theta - theta;
//std::cout << "LAHIRU\n";
        // Normalize angle
        //while (angle_error > M_PI) angle_error -= 2 * M_PI;
//std::cout << "KURUPPU\n";
        //while (angle_error < -M_PI) angle_error += 2 * M_PI;
//std::cout << "PASINDHU\n";
        float distance = std::sqrt(dx * dx + dy * dy);

        float linear = std::clamp(distance, 0.0f, 0.7f);
        float angular = std::clamp(angle_error, -0.2f, 0.2f);
//std::cout << "KOLLO\n";
        serial_.sendCommand(linear, angular);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
//std::cout << "GODA\n";
    }

    serial_.sendCommand(0, 0);
    std::cout << "[Navigator] Navigation complete\n";
}



