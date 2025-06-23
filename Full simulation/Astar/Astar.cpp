#include "Astar.hpp"
#include "send.hpp"  



std::string getKey(int x, int y) {
    std::stringstream ss;
    ss << x << "," << y;
    return ss.str();
}

// Updated isValid to work with OccupancyGrid costmap
bool isValid(int x, int y, const OccupancyGrid& grid) {
    return grid.isCellInside(x, y) && !std::isinf(grid.getCost(x, y));
}

double heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

std::vector<Node> reconstructPath(Node* node) {
    std::vector<Node> path;
    while (node != nullptr) {
        path.push_back(*node);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node> aStar(const OccupancyGrid& grid, int startX, int startY, int goalX, int goalY) {
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    double move_cost[] = {1.4, 1.0, 1.4, 1.0, 1.0, 1.4, 1.0, 1.4};

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<std::string, bool> closed;

    Node* start = new Node(startX, startY);
    start->g = 0;
    start->h = heuristic(startX, startY, goalX, goalY);
    start->f = start->g + start->h;

    open.push(*start);

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        std::string key = getKey(current.x, current.y);
        if (closed[key]) continue;
        closed[key] = true;

        if (current.x == goalX && current.y == goalY) {
            return reconstructPath(&current);
        }

        for (int i = 0; i < 8; i++) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!isValid(nx, ny, grid)) continue;

            std::string nkey = getKey(nx, ny);
            if (closed[nkey]) continue;

            Node* neighbor = new Node(nx, ny, new Node(current));
            neighbor->g = current.g + move_cost[i] * (std::isinf(grid.getCost(nx, ny)) ? 1000.0 : grid.getCost(nx, ny));
            neighbor->h = heuristic(nx, ny, goalX, goalY);
            neighbor->f = neighbor->g + neighbor->h;

            open.push(*neighbor);
        }
    }

    return {};
}

void visualizePath(const OccupancyGrid& grid, const std::vector<Node>& path, 
                   int startX, int startY, int goalX, int goalY) {
    const int cell_size = 30;
    const int rows = grid.getHeight();
    const int cols = grid.getWidth();
    
    cv::Mat image(rows * cell_size, cols * cell_size, CV_8UC3, cv::Scalar(255, 255, 255));
    
    // Draw grid lines
    for (int i = 0; i <= rows; ++i) {
        cv::line(image, cv::Point(0, i * cell_size), cv::Point(cols * cell_size, i * cell_size), 
                cv::Scalar(200, 200, 200), 1);
    }
    for (int j = 0; j <= cols; ++j) {
        cv::line(image, cv::Point(j * cell_size, 0), cv::Point(j * cell_size, rows * cell_size), 
                cv::Scalar(200, 200, 200), 1);
    }
    
    // Draw obstacles (cells with infinite cost)
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (std::isinf(grid.getCost(x, y))) {
                cv::rectangle(image, 
                             cv::Point(x * cell_size, y * cell_size),
                             cv::Point((x + 1) * cell_size, (y + 1) * cell_size),
                             cv::Scalar(0, 0, 0), cv::FILLED);
            }
        }
    }
    
    // Draw path
    if (!path.empty()) {
        for (size_t k = 0; k < path.size() - 1; ++k) {
            int x1 = path[k].x;
            int y1 = path[k].y;
            int x2 = path[k + 1].x;
            int y2 = path[k + 1].y;
            
            cv::line(image,
                    cv::Point(y1 * cell_size + cell_size/2, x1 * cell_size + cell_size/2),
                    cv::Point(y2 * cell_size + cell_size/2, x2 * cell_size + cell_size/2),
                    cv::Scalar(0, 0, 255), 3);
        }
        
        // Draw path points
        for (const auto& node : path) {
            cv::circle(image, 
                      cv::Point(node.y * cell_size + cell_size/2, node.x * cell_size + cell_size/2),
                      3, cv::Scalar(0, 255, 0), cv::FILLED);
        }
    }
    
    // Draw start and goal
    cv::circle(image, 
              cv::Point(startY * cell_size + cell_size/2, startX * cell_size + cell_size/2),
              5, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::circle(image, 
              cv::Point(goalY * cell_size + cell_size/2, goalX * cell_size + cell_size/2),
              5, cv::Scalar(0, 0, 255), cv::FILLED);
    
    // Add text labels
    cv::putText(image, "Start", 
               cv::Point(startY * cell_size + cell_size/2 + 10, startX * cell_size + cell_size/2),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(image, "Goal", 
               cv::Point(goalY * cell_size + cell_size/2 + 10, goalX * cell_size + cell_size/2),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    cv::imshow("A* Path Planning", image);
    cv::waitKey(0);
}

void calculateVelocityCommands(const std::vector<Node>& path, double initial_orientation_deg, SerialPort& serial) {
    //SerialPort serial("/dev/ttyACM0", B9600);
    
    if (path.size() < 5) {
        serial.sendCommand(0.0, 0.0); // Stop the robot
        return;
    }

    Node current = path[0];
    Node fifth_point = path[4];

    double current_x = current.y * CELL_SIZE;
    double current_y = current.x * CELL_SIZE;
    double target_x = fifth_point.y * CELL_SIZE;
    double target_y = fifth_point.x * CELL_SIZE;

    double dx = target_x - current_x;
    double dy = target_y - current_y;
    double desired_heading = atan2(dx, -dy);

    double initial_orientation_rad = (initial_orientation_deg - 180) * M_PI / 180.0;
    double angle_diff = desired_heading - initial_orientation_rad;
    
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    double distance = sqrt(dx * dx + dy * dy);
    double R = (abs(angle_diff) < 0.01) ? 1e6 : distance / (2 * sin(angle_diff));
    double omega = LINEAR_VELOCITY / R;

    double MAX_OMEGA = 1.0;
    if (abs(omega) > MAX_OMEGA) {
        omega = (omega > 0) ? MAX_OMEGA : -MAX_OMEGA;
    }

    std::cout << "Current position: (" << current.x << ", " << current.y << ")" << std::endl;
    std::cout << "5th point position: (" << fifth_point.x << ", " << fifth_point.y << ")" << std::endl;
    std::cout << "Initial orientation: " << initial_orientation_deg << " degrees" << std::endl;
    std::cout << "Desired heading: " << desired_heading * 180.0 / M_PI << " degrees" << std::endl;
    std::cout << "Angle difference: " << angle_diff * 180.0 / M_PI << " degrees" << std::endl;
    std::cout << "Distance to target: " << distance << " m" << std::endl;
    std::cout << "Turning radius (R): " << R << " m" << std::endl;
    std::cout << "Required angular velocity (ω): " << omega << " rad/s" << std::endl;
    std::cout << "Linear velocity (v): " << LINEAR_VELOCITY << " m/s" << std::endl;
    
    serial.sendCommand(LINEAR_VELOCITY, omega);
    usleep(500000);
}


// Function to create a test environment with obstacles
void createTestEnvironment(OccupancyGrid& grid) {
    // Create some walls (obstacles) using world coordinates
    // Horizontal wall from (2.5m, 2.5m) to (10.0m, 3.75m)
    for (float x = 2.5f; x < 10.0f; x += 0.05f) {
        for (float y = 2.5f; y < 3.75f; y += 0.05f) {
            std::vector<std::pair<float, float>> point;
            point.emplace_back(x, y);
            grid.updateWithGlobalPoints(point);
        }
    }
    
    // Vertical wall from (2.5m, 2.5m) to (3.75m, 10.0m)
    for (float x = 2.5f; x < 3.75f; x += 0.05f) {
        for (float y = 2.5f; y < 10.0f; y += 0.05f) {
            std::vector<std::pair<float, float>> point;
            point.emplace_back(x, y);
            grid.updateWithGlobalPoints(point);
        }
    }
    
    // Create some random obstacles
    for (int i = 0; i < 10; ++i) {
        float x = 5.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/5.0f));
        float y = 5.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/5.0f));
        
        // Create circular obstacle with 0.25m radius
        for (float angle = 0; angle < 2*M_PI; angle += 0.1f) {
            for (float r = 0; r <= 0.25f; r += 0.05f) {
                std::vector<std::pair<float, float>> point;
                point.emplace_back(x + r*cos(angle), y + r*sin(angle));
                grid.updateWithGlobalPoints(point);
            }
        }
    }
}
