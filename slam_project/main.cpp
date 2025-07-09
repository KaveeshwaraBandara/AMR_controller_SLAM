#include "LidarReader.hpp"
#include "OccupancyGrid.hpp"
#include "ICPMatcher.hpp"
#include "PoseEKF.hpp"
#include "VelocityEKF.hpp"
#include "BNO055.hpp"
#include "navigation.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include "send.hpp"
#include "Astar.hpp"
#include <unistd.h>
#include <thread>


//struct Pose2D {
//    float x = 0, y = 0, theta = 0;
//};

#define LOOP_INTERVAL_MS 500


int main() {
    LidarReader lidar("/dev/ttyUSB0", 1000000);
    if (!lidar.connect()) {
        std::cerr << "Lidar connect failed\n";
        return -1;
    }
    const char* portName = "/dev/ttyACM0"; // Adjust based on your device
    int baudRate = B9600; // Match with microcontroller's baud rate

    SerialPort serial(portName, baudRate);

    BNO055 imu;
    if (!imu.begin()) {
        std::cerr << "Failed to initialize BNO055!\n";
        return -1;
    }

    VelocityEKF velocityEKF;
    PoseEKF poseEKF;

    //EKF ekf;
    OccupancyGrid grid(500, 500, 0.05f);
    Navigator navigator(grid, poseEKF, serial);
    std::vector<Pose2D> trajectory;
    std::vector<cv::Point2f> prev_cloud;

    auto last_time = std::chrono::steady_clock::now();
    //std::string message = "set 0.03 0\n";
    //serial.sendCommand(0.03,0);

    float max_linear_velocity = 0.03f;
    float linear_vel = 0.0f;    
    //float v_icp = 0;
    //float w_icp = 0;
    
    
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

        serial.sendCommand(linear_vel,0);

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        EulerAngles orientation = imu.readEulerAngles();
        Vector3 angularVel = imu.readAngularVelocity();

        float yaw_imu = orientation.yaw * M_PI / 180.0f;
        float yaw_rate_imu = angularVel.z * M_PI / 180.0f;

      //  imu.update();
        //float v = imu.getLinearVelocity();
        //float w = imu.getAngularVelocity();

        //ekf.predict(v, w, dt);
        
        //float v = 0.03;
        //float w = 0;

//ekf.predict(v, w, dt);


        auto raw_scan = lidar.getScan();
        //ekf.predict(v, w, dt);
        auto current_cloud = toPointCloud(raw_scan);
        
        
        float v_icp = 0.0f, w_icp = 0.0f, dx = 0.0f, dy = 0.0f, dtheta = 0.0f;
        if (!prev_cloud.empty()) {
            cv::Mat Tr = runICP(prev_cloud, current_cloud);     //remembre this 2*3 matrix which i extracted in icp header file(r11......)

            float dx = Tr.at<double>(0, 2);
            float dy = Tr.at<double>(1, 2);
            float dtheta = atan2(Tr.at<double>(1, 0), Tr.at<double>(0, 0));

            //Eigen::Vector3f z;
            /*z << ekf.getState()(0) + dx,
                 ekf.getState()(1) + dy,
                 ekf.getState()(2) + dtheta;
            
            ekf.correct(z);
            Eigen::Vector3f x = ekf.getState();
            Eigen::Matrix3f P = ekf.getCovariance();
            trajectory.push_back({ x(0), x(1), x(2), P });
            */
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

        auto global_points = transformToGlobal(raw_scan, current_pose);
        grid.updateWithGlobalPoints(global_points);

        prev_cloud = current_cloud;
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_INTERVAL_MS));

        
        
        
        /*Eigen::Vector3f x = ekf.getState();
        Pose2D pose = { x(0), x(1), x(2) };
        trajectory.push_back(pose);

        auto global_points = transformToGlobal(raw_scan, pose);
        grid.updateWithGlobalPoints(global_points);
        //grid.showLiveMap(trajectory);  // OpenCV visualization

        prev_cloud = current_cloud;
       // message = "set 0.03 0\n";
	    //serial.sendData(message);
        serial.sendCommand(0.03,0);
        //if (cv::waitKey(10) == 'q') break;*/
    }

   // message = "off\n";
    //serial.sendData(message);
        serial.sendCommand(0,0);
        sleep(2);
        float yaw1 = imu.readEulerAngles().yaw * M_PI / 180.0f;
        auto turn_start = std::chrono::steady_clock::now();
        serial.sendCommand(0, 0.3);
        usleep(10472000);  // or wait for sensor feedback
        serial.sendCommand(0, 0);
        auto turn_end = std::chrono::steady_clock::now();
        float yaw2 = imu.readEulerAngles().yaw * M_PI / 180.0f;
        
        float dt_turn = std::chrono::duration<float>(turn_end - turn_start).count();
//        poseEKF.predict(0.0f, 0.3f, dt_turn);
        float dtheta = yaw2 - yaw1;
// Normalize between -π to π
//while (dtheta > M_PI) dtheta -= 2 * M_PI;
//while (dtheta < -M_PI) dtheta += 2 * M_PI;

poseEKF.predict(0.0f, dtheta / dt_turn, dt_turn);

	
	auto pose_state = poseEKF.getState();
Pose2D current_pose = { pose_state(0), pose_state(1), pose_state(2), poseEKF.getCovariance() };
trajectory.push_back(current_pose);

// Add a scan update to occupancy grid (optional, if you want map update too)
auto raw_scan = lidar.getScan();
auto global_points = transformToGlobal(raw_scan, current_pose);
grid.updateWithGlobalPoints(global_points);

// Also save previous cloud to continue ICP in next loop
std::vector<cv::Point2f> prev_cloud2;
prev_cloud2 = toPointCloud(raw_scan);
	

  grid.saveAsImageWithTrajectory("map_with_ellipse_foward.png", trajectory);
    
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

        serial.sendCommand(linear_vel,0);

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        EulerAngles orientation = imu.readEulerAngles();
        Vector3 angularVel = imu.readAngularVelocity();

        float yaw_imu = orientation.yaw * M_PI / 180.0f;
        float yaw_rate_imu = angularVel.z * M_PI / 180.0f;

      //  imu.update();
        //float v = imu.getLinearVelocity();
        //float w = imu.getAngularVelocity();

        //ekf.predict(v, w, dt);
        
        //float v = 0.03;
        //float w = 0;

//ekf.predict(v, w, dt);


        auto raw_scan = lidar.getScan();
        //ekf.predict(v, w, dt);
        auto current_cloud = toPointCloud(raw_scan);
        
        
        float v_icp = 0.0f, w_icp = 0.0f, dx = 0.0f, dy = 0.0f, dtheta = 0.0f;
PoseEKF::Vector3f pose_meas;        
if (!prev_cloud2.empty() && frame> 0) {
            cv::Mat Tr = runICP(prev_cloud2, current_cloud);     //remembre this 2*3 matrix which i extracted in icp header file(r11......)

            float dx = Tr.at<double>(0, 2);
            float dy = Tr.at<double>(1, 2);
            float dtheta = atan2(Tr.at<double>(1, 0), Tr.at<double>(0, 0));

            //Eigen::Vector3f z;
            /*z << ekf.getState()(0) + dx,
                 ekf.getState()(1) + dy,
                 ekf.getState()(2) + dtheta;
            
            ekf.correct(z);
            Eigen::Vector3f x = ekf.getState();
            Eigen::Matrix3f P = ekf.getCovariance();
            trajectory.push_back({ x(0), x(1), x(2), P });
            */
            v_icp = std::sqrt(dx*dx + dy*dy) / dt;
            w_icp = dtheta / dt;

        }else{
          dx=dy=dtheta=0;		
          pose_meas = poseEKF.getState();
          continue; 		
	}

        VelocityEKF::Vector2f vel_meas;
        vel_meas << v_icp, yaw_rate_imu;

        velocityEKF.predict(dt);
        velocityEKF.correct(vel_meas);
        auto vel_est = velocityEKF.getState();

        poseEKF.predict(vel_est(0), vel_est(1), dt);

        //PoseEKF::Vector3f pose_meas;
        if (!prev_cloud2.empty()&& frame>0) {
            auto pose_state = poseEKF.getState();
            pose_meas << pose_state(0) + dx, pose_state(1) + dy, pose_state(2) + dtheta;
        } else {
            pose_meas = poseEKF.getState();
        }

        poseEKF.correct(pose_meas);

        auto pose_state = poseEKF.getState();
        Pose2D current_pose = { pose_state(0), pose_state(1), pose_state(2), poseEKF.getCovariance() };
        trajectory.push_back(current_pose);

        auto global_points = transformToGlobal(raw_scan, current_pose);
        grid.updateWithGlobalPoints(global_points);

        prev_cloud2 = current_cloud;
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_INTERVAL_MS));

        
        
        
        /*Eigen::Vector3f x = ekf.getState();
        Pose2D pose = { x(0), x(1), x(2) };
        trajectory.push_back(pose);

        auto global_points = transformToGlobal(raw_scan, pose);
        grid.updateWithGlobalPoints(global_points);
        //grid.showLiveMap(trajectory);  // OpenCV visualization

        prev_cloud = current_cloud;
       // message = "set 0.03 0\n";
	    //serial.sendData(message);
        serial.sendCommand(0.03,0);
        //if (cv::waitKey(10) == 'q') break;*/
    }



  
  //  lidar.stop();
   // ekf.predict(0.0f, 0.3f, 10.472); // 0.3 rad/s for ~10.472 seconds
//lidar.start();  // if your driver supports restart

      /*  std::vector<cv::Point2f> prev_cloud2;

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
*/

        serial.sendCommand(0,0);
        sleep(2);
        yaw1 = imu.readEulerAngles().yaw * M_PI / 180.0f;
        turn_start = std::chrono::steady_clock::now();
        serial.sendCommand(0, 0.3);
        usleep(10472000);  // or wait for sensor feedback
        serial.sendCommand(0, 0);
        turn_end = std::chrono::steady_clock::now();
        yaw2 = imu.readEulerAngles().yaw * M_PI / 180.0f;
        
        dt_turn = std::chrono::duration<float>(turn_end - turn_start).count();
//        poseEKF.predict(0.0f, 0.3f, dt_turn);
        dtheta = yaw2 - yaw1;
// Normalize between -π to π
//while (dtheta > M_PI) dtheta -= 2 * M_PI;
//while (dtheta < -M_PI) dtheta += 2 * M_PI;

poseEKF.predict(0.0f, dtheta / dt_turn, dt_turn);
pose_state = poseEKF.getState();
 current_pose = { pose_state(0), pose_state(1), pose_state(2), poseEKF.getCovariance() };
trajectory.push_back(current_pose);

    lidar.stop();

    //update cost map
    float robot_radius = 0.3f;
    grid.updateCostMap(robot_radius);
grid.saveCostMapAsImage("cost_map.png");

// Let user select goal
grid.showCostMapWithGoalSelection();

// Access goal (optional for A*)
/*if (!grid.isGoalSelected())return 0; 
    auto [gx, gy] = grid.getSelectedGoal();
    std::cout << "[Main] Goal selected at: (" << gx << ", " << gy << ") meters\n";
    // Use gx, gy in world coordinates as target for A*

AStarPlanner planner(grid);
    auto pose_state = poseEKF.getState();
    int start_x = static_cast<int>(pose_state(0) / grid.getResolution()) + grid.getOriginX();
    int start_y = static_cast<int>(pose_state(1) / grid.getResolution()) + grid.getOriginY();
    int goal_x = static_cast<int>(gx / grid.getResolution()) + grid.getOriginX();
    int goal_y = static_cast<int>(gy / grid.getResolution()) + grid.getOriginY();

    auto path = planner.plan(start_x, start_y, goal_x, goal_y);

    int width = grid.getWidth(), height = grid.getHeight();
    cv::Mat map_img(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float cost = grid.getCost(x, y);
            uchar val = std::isinf(cost) ? 0 : static_cast<uchar>(255 * (1.0f - std::min(cost, 1.0f)));
            map_img.at<cv::Vec3b>(y, x) = cv::Vec3b(val, val, val);
        }
    }

    AStarPlanner::drawPath(map_img, path, cv::Scalar(0, 255, 0));
    cv::circle(map_img, cv::Point(goal_x, goal_y), 4, cv::Scalar(0, 0, 255), -1);
    cv::circle(map_img, cv::Point(start_x, start_y), 4, cv::Scalar(255, 0, 0), -1);

    cv::imwrite("a_star_path.png", map_img);
    cv::imshow("A* Path", map_img);
    cv::waitKey(0);
*/


//navigator.setGoal();               // Click goal on cost map
//navigator.runNavigationLoop();     // Run full path + control loop

while (true) {
    // --- SLAM update: ICP + EKF ---
    auto raw_scan = lidar.getScan();
    auto current_cloud = toPointCloud(raw_scan);

    float dx = 0, dy = 0, dtheta = 0;
    float v_icp = 0, w_icp = 0;

    if (!prev_cloud.empty()) {
        cv::Mat Tr = runICP(prev_cloud, current_cloud);
        dx = Tr.at<double>(0, 2);
        dy = Tr.at<double>(1, 2);
        dtheta = atan2(Tr.at<double>(1, 0), Tr.at<double>(0, 0));

        v_icp = std::sqrt(dx * dx + dy * dy) / slam_dt;
        w_icp = dtheta / slam_dt;
    }

    EulerAngles orientation = imu.readEulerAngles();
    Vector3 angularVel = imu.readAngularVelocity();
    float yaw_rate_imu = angularVel.z * M_PI / 180.0f;

    // Velocity EKF
    VelocityEKF::Vector2f vel_meas;
    vel_meas << v_icp, yaw_rate_imu;
    velocityEKF.predict(slam_dt);
    velocityEKF.correct(vel_meas);
    auto vel_est = velocityEKF.getState();

    // Pose EKF
    poseEKF.predict(vel_est(0), vel_est(1), slam_dt);

    PoseEKF::Vector3f pose_meas;
    auto pose_state = poseEKF.getState();
    pose_meas << pose_state(0) + dx, pose_state(1) + dy, pose_state(2) + dtheta;
    poseEKF.correct(pose_meas);

    pose_state = poseEKF.getState();
    Pose2D current_pose = { pose_state(0), pose_state(1), pose_state(2) };
    prev_cloud = current_cloud;

    // Update occupancy grid
    auto global_pts = transformToGlobal(raw_scan, current_pose);
    grid.updateWithGlobalPoints(global_pts);

    // --- Path Planning ---
    int curr_x = static_cast<int>(current_pose.x / grid.getResolution()) + grid.getOriginX();
    int curr_y = static_cast<int>(current_pose.y / grid.getResolution()) + grid.getOriginY();

    auto path = planner.plan(curr_x, curr_y, goal_x, goal_y);
    if (path.empty()) {
        std::cerr << "Path planning failed!\n";
        serial.sendCommand(0.0, 0.0);
        break;
    }

    // --- Velocity Command Generation ---
    computeAndSendCommand(curr_x, curr_y, goal_x, goal_y, path);

    // --- Goal check ---
    float goal_x_m = (goal_x - grid.getOriginX()) * grid.getResolution();
    float goal_y_m = (goal_y - grid.getOriginY()) * grid.getResolution();
    float dist_to_goal = std::hypot(pose_state(0) - goal_x_m, pose_state(1) - goal_y_m);
    if (dist_to_goal < 0.15f) {
        std::cout << "Goal Reached!\n";
        serial.sendCommand(0.0, 0.0);
        break;
    }

    usleep(300000);  // 300 ms
}


    
    
    
    /*grid.updateCostMap(robot_radius);
grid.saveCostMapAsImage("cost_map.png");
grid.showCostMapWithClick(); // <-- Add this line*/

    //grid.showCostMap();        

//cv::imwrite("final_cost_map.png", cv::imread("Cost Map"));  
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
    grid.saveAsImageWithTrajectory("map_with_ellipse_full.png", trajectory);
    
   /* int goalX, goalY;
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
    //cv::destroyAllWindows();*/
    return 0;
}

