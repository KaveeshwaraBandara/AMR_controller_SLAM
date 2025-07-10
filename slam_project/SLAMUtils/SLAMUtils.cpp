#include "SLAMUtils.hpp"

#include <cmath>            // for M_PI, atan2, std::sqrt
#include <vector>           // for std::vector
#include <opencv2/core.hpp> // for cv::Point2f, cv::Mat
#include "ICPMatcher.hpp"   // for runICP
//#include "transform.hpp"    // if transformToGlobal is in a helper file
//#include "Pose2D.hpp"       // for Pose2D struct definition

using namespace std; // optional
using namespace cv;  // optional

static std::vector<cv::Point2f> toPointCloud(const std::vector<std::pair<float, float>>& scan) {
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



void updateSLAM(const std::vector<std::pair<float, float>>& scan,
    const EulerAngles& orientation,
    const Vector3& angularVel,
    std::vector<cv::Point2f>& prev_cloud,
    VelocityEKF& velocityEKF,
    PoseEKF& poseEKF,
    std::vector<Pose2D>& trajectory,
    OccupancyGrid& grid,
    float dt)
{
float yaw_imu = orientation.yaw * M_PI / 180.0f;
float yaw_rate_imu = angularVel.z * M_PI / 180.0f;

auto current_cloud = toPointCloud(scan);

float v_icp = 0.0f, w_icp = 0.0f, dx = 0.0f, dy = 0.0f, dtheta = 0.0f;

if (!prev_cloud.empty()) {
cv::Mat Tr = runICP(prev_cloud, current_cloud);

dx = Tr.at<double>(0, 2);
dy = Tr.at<double>(1, 2);
dtheta = atan2(Tr.at<double>(1, 0), Tr.at<double>(0, 0));

v_icp = std::sqrt(dx*dx + dy*dy) / dt;
w_icp = dtheta / dt;
}

VelocityEKF::Vector2f vel_meas;
vel_meas << v_icp, yaw_rate_imu;

velocityEKF.predict(dt);
velocityEKF.correct(vel_meas);
auto vel_est = velocityEKF.getState();

poseEKF.predict(vel_est(0), vel_est(1), dt);

PoseEKF::Vector3f pose_meas;
if (!prev_cloud.empty()) {
auto pose_state = poseEKF.getState();
pose_meas << pose_state(0) + dx, pose_state(1) + dy, pose_state(2) + dtheta;
} else {
pose_meas = poseEKF.getState();
}

poseEKF.correct(pose_meas);

auto pose_state = poseEKF.getState();
Pose2D current_pose = { pose_state(0), pose_state(1), pose_state(2), poseEKF.getCovariance() };
trajectory.push_back(current_pose);

auto global_points = transformToGlobal(scan, current_pose);
grid.updateWithGlobalPoints(global_points);

prev_cloud = current_cloud;
}


void drawNavigationDebug(const OccupancyGrid& grid,
    const std::vector<std::pair<int, int>>& path,
    const Pose2D& robot_pose,
    int goal_x, int goal_y)
{
cv::Mat debug_img = grid.toImage();

// Draw path
for (const auto& [x, y] : path) {
cv::circle(debug_img, cv::Point(x, y), 1, cv::Scalar(255, 255, 0), -1);
}

// Draw robot pose
int robot_x = static_cast<int>(robot_pose.x / grid.getResolution()) + grid.getOriginX();
int robot_y = static_cast<int>(robot_pose.y / grid.getResolution()) + grid.getOriginY();
cv::circle(debug_img, cv::Point(robot_x, robot_y), 4, cv::Scalar(0, 255, 0), -1);

// Heading line
int line_length = 10;
int end_x = static_cast<int>(robot_x + line_length * std::cos(robot_pose.theta));
int end_y = static_cast<int>(robot_y + line_length * std::sin(robot_pose.theta));
cv::line(debug_img, cv::Point(robot_x, robot_y), cv::Point(end_x, end_y), cv::Scalar(0, 255, 0), 1);

// Draw goal
cv::circle(debug_img, cv::Point(goal_x, goal_y), 4, cv::Scalar(0, 0, 255), -1);

// Display
cv::imshow("SLAM Navigation Debug", debug_img);
cv::waitKey(1);
}
