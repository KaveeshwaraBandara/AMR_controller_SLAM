#include "LidarReader.hpp"
#include "OccupancyGrid.hpp"
#include "ICPMatcher.hpp"
#include "EKF.hpp"
#include "IMUReader.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include "send.hpp"
#include "Astar.hpp"
#include <unistd.h>

//struct Pose2D {
//    float x = 0, y = 0, theta = 0;
//};

#define LOOP_INTERVAL_MS 500

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

   // IMUReader imu;
    //imu.initialize();

    EKF ekf;
    OccupancyGrid grid(250, 250, 0.05f);
    std::vector<Pose2D> trajectory;
    std::vector<cv::Point2f> prev_cloud;

    auto last_time = std::chrono::steady_clock::now();
    //std::string message = "set 0.03 0\n";
    serial.sendCommand(0.03,0);

    for (int frame = 0; frame < 100; ++frame) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

      //  imu.update();
        //float v = imu.getLinearVelocity();
        //float w = imu.getAngularVelocity();

        //ekf.predict(v, w, dt);
        
        float v = 0.03;
        float w = 0;

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
       // message = "set 0.03 0\n";
	//serial.sendData(message);
serial.sendCommand(0.03,0);
        //if (cv::waitKey(10) == 'q') break;
    }

   // message = "off\n";
    //serial.sendData(message);
    serial.sendCommand(0,0);
    sleep(2);
auto turn_start = std::chrono::steady_clock::now();
serial.sendCommand(0, 0.3);
usleep(10472000);  // or wait for sensor feedback
serial.sendCommand(0, 0);
auto turn_end = std::chrono::steady_clock::now();

float dt_turn = std::chrono::duration<float>(turn_end - turn_start).count();
ekf.predict(0.0f, 0.3f, dt_turn);

    
  //  lidar.stop();
   // ekf.predict(0.0f, 0.3f, 10.472); // 0.3 rad/s for ~10.472 seconds
//lidar.start();  // if your driver supports restart

std::vector<cv::Point2f> prev_cloud2;

for (int frame = 0; frame < 100; ++frame) {
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_time).count();
    last_time = now;

    float v = 0.03f, w = 0.0f;
    ekf.predict(v, w, dt);
    auto raw_scan = lidar.getScan();
    auto current_cloud = toPointCloud(raw_scan);

    if (!prev_cloud2.empty()) {
        cv::Mat Tr = runICP(prev_cloud2, current_cloud);
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
    prev_cloud2 = current_cloud;

    serial.sendCommand(v, w);
}

   serial.sendCommand(0,0);
    sleep(2);
turn_start = std::chrono::steady_clock::now();
serial.sendCommand(0, 0.3);
usleep(10472000);  // or wait for sensor feedback
serial.sendCommand(0, 0);
turn_end = std::chrono::steady_clock::now();

dt_turn = std::chrono::duration<float>(turn_end - turn_start).count();
ekf.predict(0.0f, 0.3f, dt_turn);

    
    lidar.stop();

    //update cost map
    float robot_radius = 0.3f;
    grid.updateCostMap(robot_radius);
    grid.showCostMap();        

cv::imwrite("final_cost_map.png", cv::imread("Cost Map"));  
    // Create cost map visualization
    //cv::Mat cost_map_img(grid.getHeight(), grid.getWidth(), CV_8UC3, cv::Scalar(255, 255, 255));
    
    // Draw cost map
  /*  for (int y = 0; y < grid.getHeight(); ++y) {
        for (int x = 0; x < grid.getWidth(); ++x) {
            float cost = grid.getCost(x, y);
            if (std::isinf(cost)) {
                cost_map_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Red for obstacles
            }
        }
    }*/
    grid.saveAsImageWithTrajectory("map_with_ellipse.png", trajectory);
    
    int goalX, goalY;
    std::cout << "\nEnter goal cell coordinates (x y in grid units [0-" << grid.getWidth()-1 << ", 0-" << grid.getHeight()-1 << "]): ";
    std::cin >> goalX >> goalY;

  if (!grid.isInside(goalX, goalY)) {
      std::cerr << "Goal is outside map bounds.\n";
      return -1;
    }

    if (std::isinf(grid.getCost(goalX, goalY))) {
      std::cerr << "Goal cell is in an obstacle. Choose a free space.\n";
      return -1;
    }

    std::cout << "Goal set at (" << goalX << ", " << goalY << "). You can now run path planning.\n";
    
    int startX = ekf.getState()(0); 
    int startY = ekf.getState()(1);

    std::vector<float> costmap = grid.getCostMap();
    
    // A* planned path
std::vector<Node> path = aStar(grid, startY, startX, goalY, goalX);
if (path.empty()) {
    std::cout << "No path found!\n";
    return -1;
}
visualizePath(grid, path, startY, startX, goalY, goalX);

// Navigation loop
for (size_t i = 0; i + 1 < path.size(); i += 4) {
    // Current EKF pose
    Eigen::Vector3f pose = ekf.getState();
    int curr_x = static_cast<int>(pose(0) / grid.getResolution()) + grid.getOriginX();
    int curr_y = static_cast<int>(pose(1) / grid.getResolution()) + grid.getOriginX();

    // Target cell (4 steps ahead or final point)
    Node target = path[std::min(i + 4, path.size() - 1)];

    // Convert to meters
    float target_x_m = (target.y - grid.getOriginX()) * grid.getResolution();
    float target_y_m = (target.x - grid.getOriginX()) * grid.getResolution();

    float dx = target_x_m - pose(0);
    float dy = target_y_m - pose(1);
    float dist = std::sqrt(dx * dx + dy * dy);
    float heading_to_target = std::atan2(dy, dx);
    float angle_diff = heading_to_target - pose(2);

    // Normalize angle
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    float v = 0.03f;
    float w = angle_diff;

    // Clamp angular velocity
    if (w > 1.0f) w = 1.0f;
    if (w < -1.0f) w = -1.0f;

    // Send command and update EKF
    serial.sendCommand(v, w);
    usleep(500000);  // half-second movement

    ekf.predict(v, w, 0.5f);
    Eigen::Vector3f updated = ekf.getState();
    Eigen::Matrix3f P = ekf.getCovariance();
    trajectory.push_back({ updated(0), updated(1), updated(2), P });

    // Optional: stop if very close to goal
    if (i + 4 >= path.size() - 1 && dist < 0.15f) {
        std::cout << "Reached goal!\n";
        break;
    }
}

serial.sendCommand(0.0, 0.0);
lidar.stop();
grid.saveAsImageWithTrajectory("final_map_with_trajectory.png", trajectory);

    //std::vector<Node> path = aStar(costmap, startX, startY, goalX, goalY);

    /*if (path.empty()) {
        std::cout << "No path found!" << std::endl;
    } else {
        double initial_orientation_deg = ekf.getState()(2);
        calculateVelocityCommands(path, initial_orientation_deg);
    }*/

    //grid.saveAsImageWithTrajectory("map_with_heading.png", trajectory);
    //cv::destroyAllWindows();
    return 0;
}

