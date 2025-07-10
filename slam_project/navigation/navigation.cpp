#include "navigation.hpp"
#include <cmath>
#include <unistd.h>
#include <iostream>
#include "SLAMUtils.hpp"
#include "Trajviz.hpp"

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


void Navigator::setGoal(float gx, float gy) {
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
float distance = std::sqrt(dx * dx + dy * dy);
float desired_theta = std::atan2(dy, dx);
float angle_error = desired_theta - curr_theta;

// Normalize to [-π, π]
while (angle_error > M_PI) angle_error -= 2 * M_PI;
while (angle_error < -M_PI) angle_error += 2 * M_PI;

float v = 0.0f, w = 0.0f;

// Turn in place if angle error is large
if (std::abs(angle_error) > 0.6f) {
v = 0.0f;
w = std::clamp(angle_error, -0.4f, 0.4f);
} else {
// Smooth forward motion with angular adjustment
float v_scale = std::cos(angle_error);  // reduce speed on turns
v = std::clamp(distance * v_scale, 0.0f, 0.03f);
w = std::clamp(angle_error, -0.06f, 0.06f);
}

serial_.sendCommand(v, w);
std::cout << "[Command] v: " << v << " m/s, w: " << w << " rad/s\n";
}


void Navigator::navigateToGoal() {
    if (!goal_set_) return;

    std::cout << "[Navigator] Starting navigation...\n";
prev_cloud_.clear();

std::cout << "[SLAM] prev_cloud size at entry: " << prev_cloud_.size() << "\n";


constexpr float goal_tolerance = 0.1f;     // meters
    constexpr float angle_tolerance = 0.3f;    // radians
    constexpr int max_steps = 500;
    constexpr int step_delay_ms = 500;

    auto last_time = std::chrono::steady_clock::now();
    for (int step = 0; step < max_steps; ++step) {  // limit for demo
        // Get current time
        
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
       /* if (path.size() < 2) {
            std::cout << "[Navigator] Goal reached\n";
            serial_.sendCommand(0, 0);
            break;
        }*/
//std::cout << "HELLO\n";
       /* auto [next_x, next_y] = path[1];
        float target_world_x = (next_x - grid_.getOriginX()) * grid_.getResolution();
        float target_world_y = (next_y - grid_.getOriginY()) * grid_.getResolution();*/
//std::cout << "SAHAN\n";
        /*float dx = target_world_x - x;
        float dy = target_world_y - y;
        float desired_theta = atan2(dy, dx);
        float angle_error = desired_theta - theta;*/
//std::cout << "LAHIRU\n";
        // Normalize angle
        //while (angle_error > M_PI) angle_error -= 2 * M_PI;
//std::cout << "KURUPPU\n";
        //while (angle_error < -M_PI) angle_error += 2 * M_PI;
//std::cout << "PASINDHU\n";
        /*float distance = std::sqrt(dx * dx + dy * dy);

        float linear = std::clamp(distance, 0.0f, 0.7f);
        float angular = std::clamp(angle_error, -0.2f, 0.2f);*/
//std::cout << "KOLLO\n";
        //serial_.sendCommand(linear, angular);


        // If close to goal
        float goal_world_x = (goal_x_ - grid_.getOriginX()) * grid_.getResolution();
        float goal_world_y = (goal_y_ - grid_.getOriginY()) * grid_.getResolution();
        float dist_to_goal = std::hypot(goal_world_x - x, goal_world_y - y);
        float angle_to_goal = std::atan2(goal_world_y - y, goal_world_x - x);
        float angle_error = angle_to_goal - theta;


        if (dist_to_goal < goal_tolerance && std::abs(angle_error) < angle_tolerance) {
            std::cout << "[Navigator] Goal reached within tolerance\n";
            serial_.sendCommand(0, 0);
            break;
        }

        // Choose a further target to avoid jerky motion
        //size_t next_index = std::min<size_t>(2, path.size() - 1);
        size_t next_index = std::min<size_t>(2, std::max<size_t>(1, path.size() - 1));

        auto [next_x, next_y] = path[next_index];
        float target_world_x = (next_x - grid_.getOriginX()) * grid_.getResolution();
        float target_world_y = (next_y - grid_.getOriginY()) * grid_.getResolution();

        computeAndSendCommand(x, y, theta, target_world_x, target_world_y);



        std::this_thread::sleep_for(std::chrono::milliseconds(step_delay_ms));
//std::cout << "GODA\n";
    }

    serial_.sendCommand(0, 0);
    std::cout << "[Navigator] Navigation complete\n";
    TrajectoryVisualizer::drawTrajectoryOnMap("cost_map_debugviz.png", trajectory_, grid_, {goal_x_, goal_y_});
}



