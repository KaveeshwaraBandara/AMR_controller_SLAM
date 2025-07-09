// navigation/Navigator.cpp

#include "navigation.hpp"
#include <cmath>
#include <unistd.h>
#include <iostream>

Navigator::Navigator(OccupancyGrid& grid, PoseEKF& ekf, SerialPort& serial)
    : grid_(grid), poseEKF_(ekf), serial_(serial), planner_(grid) {}

void Navigator::setGoal() {
    grid_.showCostMapWithGoalSelection();
    if (grid_.isGoalSelected()) {
        std::tie(goal_x_, goal_y_) = grid_.getSelectedGoal();
        goal_set_ = true;
        std::cout << "[Navigator] Goal selected at (" << goal_x_ << ", " << goal_y_ << ")\n";
    } else {
        std::cerr << "[Navigator] No goal selected.\n";
        goal_set_ = false;
    }
}

void Navigator::setGoal(int gx, int gy) {
    goal_x_ = gx;
    goal_y_ = gy;
    goal_set_ = true;
}

float Navigator::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void Navigator::computeAndSendCommand(int curr_x, int curr_y, int goal_x, int goal_y, const std::vector<std::pair<int, int>>& path) {
    if (path.size() < 2) return;

    // Find a lookahead point
    std::pair<int, int> target = path[std::min<size_t>(4, path.size() - 1)];

    float target_x_m = (target.first - grid_.getOriginX()) * grid_.getResolution();
    float target_y_m = (target.second - grid_.getOriginY()) * grid_.getResolution();

    auto state = poseEKF_.getState();
    float curr_x_m = state(0);
    float curr_y_m = state(1);
    float theta = state(2);

    float dx = target_x_m - curr_x_m;
    float dy = target_y_m - curr_y_m;
    float heading = std::atan2(dy, dx);
    float angle_diff = normalizeAngle(heading - theta);
    float distance = std::sqrt(dx * dx + dy * dy);

    float v = std::min(0.03f, distance);  // Optional: scale velocity by distance
    float w = angle_diff;

    // Clamp angular velocity
    if (w > 1.0f) w = 1.0f;
    if (w < -1.0f) w = -1.0f;

    serial_.sendCommand(v, w);
}

void Navigator::runNavigationLoop() {
    if (!goal_set_) {
        std::cerr << "[Navigator] No goal set. Aborting.\n";
        return;
    }

    while (true) {
      //updatePoseFromSLAM();
        auto state = poseEKF_.getState();
        int curr_x = static_cast<int>(state(0) / grid_.getResolution()) + grid_.getOriginX();
        int curr_y = static_cast<int>(state(1) / grid_.getResolution()) + grid_.getOriginY();

        auto path = planner_.plan(curr_x, curr_y, goal_x_, goal_y_);
        if (path.empty()) {
            std::cerr << "[Navigator] Path planning failed.\n";
            serial_.sendCommand(0.0, 0.0);
            break;
        }

        computeAndSendCommand(curr_x, curr_y, goal_x_, goal_y_, path);

        float goal_x_m = (goal_x_ - grid_.getOriginX()) * grid_.getResolution();
        float goal_y_m = (goal_y_ - grid_.getOriginY()) * grid_.getResolution();
        float dist_to_goal = std::hypot(state(0) - goal_x_m, state(1) - goal_y_m);

        if (dist_to_goal < 0.15f) {
            std::cout << "[Navigator] Goal reached!\n";
            serial_.sendCommand(0.0, 0.0);
            break;
        }

        usleep(300000); // 300 ms delay between steps
    }
}



/*static std::vector<cv::Point2f> toPointCloud(const std::vector<std::pair<float, float>>& scan) {
    std::vector<cv::Point2f> cloud;
    for (auto [angle_deg, dist_m] : scan) {
        float theta = angle_deg * CV_PI / 180.0f;
        cloud.emplace_back(dist_m * cos(theta), dist_m * sin(theta));
    }
    return cloud;
}

static std::vector<std::pair<float, float>> transformToGlobal(const std::vector<std::pair<float, float>>& scan, const Pose2D& pose) {
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



void Navigator::updatePoseFromSLAM() {
    auto raw_scan = lidar_.getScan();
    auto current_cloud = toPointCloud(raw_scan);

    float v_icp = 0.0f, w_icp = 0.0f, dx = 0.0f, dy = 0.0f, dtheta = 0.0f;
    PoseEKF::Vector3f pose_meas;

    if (!prev_cloud_.empty()) {
        cv::Mat Tr = runICP(prev_cloud_, current_cloud);
        dx = Tr.at<double>(0, 2);
        dy = Tr.at<double>(1, 2);
        dtheta = atan2(Tr.at<double>(1, 0), Tr.at<double>(0, 0));

        v_icp = std::sqrt(dx*dx + dy*dy) / slam_dt_;
        w_icp = dtheta / slam_dt_;
    }

    EulerAngles orientation = imu_.readEulerAngles();
    Vector3 angularVel = imu_.readAngularVelocity();
    float yaw_rate_imu = angularVel.z * M_PI / 180.0f;

    // Velocity EKF
    VelocityEKF::Vector2f vel_meas;
    vel_meas << v_icp, yaw_rate_imu;

    velocityEKF_.predict(slam_dt_);
    velocityEKF_.correct(vel_meas);
    auto vel_est = velocityEKF_.getState();

    // Pose EKF
    poseEKF_.predict(vel_est(0), vel_est(1), slam_dt_);

    if (!prev_cloud_.empty()) {
        auto pose_state = poseEKF_.getState();
        pose_meas << pose_state(0) + dx, pose_state(1) + dy, pose_state(2) + dtheta;
    } else {
        pose_meas = poseEKF_.getState();
    }

    poseEKF_.correct(pose_meas);
    prev_cloud_ = current_cloud;

    // Update cost map if needed
    auto global_pts = transformToGlobal(raw_scan, {
        poseEKF_.getState()(0),
        poseEKF_.getState()(1),
        poseEKF_.getState()(2)
    });
    grid_.updateWithGlobalPoints(global_pts);
}
*/
