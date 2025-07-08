#include "OccupancyGrid.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

OccupancyGrid::OccupancyGrid(int width, int height, float resolution, float origin_x, float origin_y)
    : width_(width), height_(height), resolution_(resolution) {
    
    log_odds_grid_.resize(width_ * height_, 0.0f);
    origin_x_ = static_cast<int>(origin_x / resolution_) + width_ / 2;
    origin_y_ = static_cast<int>(origin_y / resolution_) + height_ / 2;
}
   

int OccupancyGrid::getOriginX() const { return origin_x_; }
int OccupancyGrid::getOriginY() const { return origin_y_; }



bool OccupancyGrid::isInside(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

float OccupancyGrid::getLogOdds(int x, int y) const {
    if (!isInside(x, y)) return 0.0f;
    return log_odds_grid_[y * width_ + x];
}

void OccupancyGrid::setLogOdds(int x, int y, float delta) {
    if (!isInside(x, y)) return;
    float& val = log_odds_grid_[y * width_ + x];
    val = std::clamp(val + delta, log_odds_min_, log_odds_max_);
}

void OccupancyGrid::raycastAndUpdate(int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (x0 != x1 || y0 != y1) {
        setLogOdds(x0, y0, log_odds_miss_);
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void OccupancyGrid::updateWithScan(const std::vector<std::pair<float, float>>& scan) {
    for (const auto& [angle_deg, dist_m] : scan) {
        float angle_rad = angle_deg * M_PI / 180.0f;
        float x = dist_m * cos(angle_rad);
        float y = dist_m * sin(angle_rad);

        int gx = static_cast<int>(x / resolution_) + origin_x_;
        int gy = static_cast<int>(y / resolution_) + origin_y_;

        raycastAndUpdate(origin_x_, origin_y_, gx, gy);
        setLogOdds(gx, gy, log_odds_hit_);
    }
}


void OccupancyGrid::saveAsImage(const std::string& filename) {
    cv::Mat image(height_, width_, CV_8UC1);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            float val = getLogOdds(x, y);
            uint8_t pixel;

            if (val > 1.0f)
                pixel = 0;        // occupied → black
            else if (val < -1.0f)
                pixel = 255;      // free → white
            else
                pixel = 128;      // unknown → gray

            image.at<uchar>(y, x) = pixel;
        }
    }

    cv::imwrite(filename, image);
    std::cout << "[Info] Map saved as image: " << filename << std::endl;
}


// Simple console visualization
void printGrid(const OccupancyGrid& grid, int width, int height) {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float val = grid.getLogOdds(x, y);
            if (val > 1.0f)
                std::cout << "#";   // occupied
            else if (val < -1.0f)
                std::cout << ".";   // free
            else
                std::cout << " ";   // unknown
        }
        std::cout << "\n";
    }
}

std::vector<cv::Point2f> convertToPoint2f(const std::vector<std::pair<float, float>>& scan) {
    std::vector<cv::Point2f> points;
    for (const auto& [angle_deg, dist_m] : scan) {
        float rad = angle_deg * CV_PI / 180.0f;
        float x = dist_m * cos(rad);
        float y = dist_m * sin(rad);
        points.emplace_back(x, y);
    }
    return points;
}


void OccupancyGrid::updateWithGlobalPoints(const std::vector<std::pair<float, float>>& points) {
    for (const auto& [x, y] : points) {
        int gx = static_cast<int>(x / resolution_) + origin_x_;
        int gy = static_cast<int>(y / resolution_) + origin_y_;
        if (!isInside(gx, gy)) continue;

        raycastAndUpdate(origin_x_, origin_y_, gx, gy);  // Mark free
        setLogOdds(gx, gy, log_odds_hit_);               // Mark occupied
    }
}


void OccupancyGrid::saveAsImageWithTrajectory(const std::string& filename, const std::vector<Pose2D>& trajectory) {
    cv::Mat image(height_, width_, CV_8UC3); // 3-channel grayscale

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            float val = getLogOdds(x, y);
            uchar pixel;
            if (val > 1.0f)
                pixel = 0;
            else if (val < -1.0f)
                pixel = 255;
            else
                pixel = 128;
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(pixel, pixel, pixel);
        }
    }

    // Draw pose arrows
/*    for (const auto& pose : trajectory) {
        int x0 = static_cast<int>(pose.x / resolution_) + origin_x_;
        int y0 = static_cast<int>(pose.y / resolution_) + origin_y_;

        if (!isInside(x0, y0)) continue;

        float arrow_len = 10.0f;  // pixels
        int x1 = static_cast<int>(x0 + arrow_len * std::cos(pose.theta));
        int y1 = static_cast<int>(y0 + arrow_len * std::sin(pose.theta));

        cv::arrowedLine(image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 0, 255), 1);
    }*/
    
    for (const auto& pose : trajectory) {
    int x0 = static_cast<int>(pose.x / resolution_) + origin_x_;
    int y0 = static_cast<int>(pose.y / resolution_) + origin_y_;

    if (!isInside(x0, y0)) continue;

    float arrow_len = 10.0f;
    int x1 = static_cast<int>(x0 + arrow_len * std::cos(pose.theta));
    int y1 = static_cast<int>(y0 + arrow_len * std::sin(pose.theta));

    cv::arrowedLine(image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 0, 255), 1);

    // Draw uncertainty ellipse from 2x2 covariance submatrix
    Eigen::Matrix2f cov2d = pose.covariance.block<2,2>(0,0);
    drawCovarianceEllipse(image, x0, y0, cov2d, cv::Scalar(255, 0, 0));
}


    cv::imwrite(filename, image);
    std::cout << "[Info] Saved map with trajectory and headings: " << filename << std::endl;
}


void OccupancyGrid::showLiveMap(const std::vector<Pose2D>& trajectory) {
    cv::Mat image(height_, width_, CV_8UC3);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            float val = getLogOdds(x, y);
            uchar pixel;
            if (val > 1.0f)
                pixel = 0;
            else if (val < -1.0f)
                pixel = 255;
            else
                pixel = 128;
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(pixel, pixel, pixel);
        }
    }

    for (const auto& pose : trajectory) {
        int x0 = static_cast<int>(pose.x / resolution_) + origin_x_;
        int y0 = static_cast<int>(pose.y / resolution_) + origin_y_;

        if (!isInside(x0, y0)) continue;

        float arrow_len = 10.0f;
        int x1 = static_cast<int>(x0 + arrow_len * std::cos(pose.theta));
        int y1 = static_cast<int>(y0 + arrow_len * std::sin(pose.theta));

        cv::arrowedLine(image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 0, 255), 1);
    }

    cv::imshow("Live Occupancy Map", image);
    cv::waitKey(1);  // Small delay to refresh window
}


void OccupancyGrid::drawCovarianceEllipse(cv::Mat& img, int cx, int cy, const Eigen::Matrix2f& cov, const cv::Scalar& color) {
    cv::Size axes;
    double angle;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
    Eigen::Vector2f eigenvalues = eig.eigenvalues();
    Eigen::Matrix2f eigenvectors = eig.eigenvectors();

    float a = std::sqrt(eigenvalues[1]) * 2.0f;  // major axis
    float b = std::sqrt(eigenvalues[0]) * 2.0f;  // minor axis

    angle = std::atan2(eigenvectors(1, 1), eigenvectors(0, 1)) * 180.0 / CV_PI;

    axes.width = static_cast<int>(a * 10); // scale for visibility
    axes.height = static_cast<int>(b * 10);

    cv::ellipse(img, cv::Point(cx, cy), axes, angle, 0, 360, color, 1);
}

bool OccupancyGrid::isFree(int x, int y) const {
    if (!isInside(x, y)) return false;
    float log_odds = getLogOdds(x, y);
    return log_odds < 0.0f;  // Free if log_odds < -1.0
}

/*void OccupancyGrid::updateCostMap(float robot_radius) {
    // Clear and initialize cost map
    cost_map_.assign(width_ * height_, std::numeric_limits<float>::infinity());
    robot_radius_cells_ = robot_radius / resolution_;

    // Mark free cells with 0 cost
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (isFree(x, y)) {
                cost_map_[y * width_ + x] = 0.0f;
            }
        }
    }

    // Inflate obstacles
    inflateObstacles(robot_radius_cells_);
}*/


void OccupancyGrid::updateCostMap(float robot_radius) {
    cost_map_.assign(width_ * height_, std::numeric_limits<float>::infinity());
    robot_radius_cells_ = robot_radius / resolution_;

    // First pass: set free cells to 0
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (isFree(x, y)) {
                cost_map_[y * width_ + x] = 0.0f;
            }
        }
    }

    // Inflate obstacles with gradient cost
    int radius_int = static_cast<int>(std::ceil(robot_radius_cells_));
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (isOccupied(x, y)) {
                for (int dy = -radius_int; dy <= radius_int; ++dy) {
                    for (int dx = -radius_int; dx <= radius_int; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (!isInside(nx, ny)) continue;

                        float dist = std::sqrt(dx * dx + dy * dy);
                        if (dist > radius_int) continue;

                        float cost = std::max(1.0f - (dist / robot_radius_cells_), 0.0f);
                        cost_map_[ny * width_ + nx] = std::min(cost_map_[ny * width_ + nx], cost);
                    }
                }
            }
        }
    }
}


void OccupancyGrid::inflateObstacles(float radius_cells) {
    std::vector<bool> obstacle_grid(width_ * height_, false);
    
    // First pass: identify all obstacle cells
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (!isOccupied(x, y)) {  // Obstacle or unknown
                obstacle_grid[y * width_ + x] = true;
            }
        }
    }

    // Second pass: inflate obstacles
    int radius_int = static_cast<int>(std::ceil(radius_cells));
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (obstacle_grid[y * width_ + x]) {
                // Inflate this obstacle
                for (int dy = -radius_int; dy <= radius_int; ++dy) {
                    for (int dx = -radius_int; dx <= radius_int; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (isInside(nx, ny) && (dx*dx + dy*dy <= radius_int*radius_int)) {
                            cost_map_[ny * width_ + nx] = std::numeric_limits<float>::infinity();
                        }
                    }
                }
            }
        }
    }
}

const std::vector<float>& OccupancyGrid::getCostMap() const {
    return cost_map_;
}

float OccupancyGrid::getCost(int x, int y) const {
    if (!isInside(x, y)) return std::numeric_limits<float>::infinity();
    return cost_map_[y * width_ + x];
}

void OccupancyGrid::showCostMap() const {
    cv::Mat image(height_, width_, CV_8UC1);
    
    float max_cost = 1.0f; // For visualization scaling
    
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            float cost = cost_map_[y * width_ + x];
            if (std::isinf(cost)) {
                image.at<uchar>(y, x) = 0; // Black for obstacles/infinity
            } else {
                // Scale the cost for visualization
                uchar value = static_cast<uchar>(255 * (1.0f - std::min(cost/max_cost, 1.0f)));
                image.at<uchar>(y, x) = value;
            }
        }
    }
    
    for (int y = 0; y < height_; y += 20) {
      for (int x = 0; x < width_; x += 20) {
        std::string label = std::to_string(x) + "," + std::to_string(y);
        cv::putText(image, label, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.3, 100, 1);
    }
  }

    
    cv::imshow("Cost Map", image);
    cv::waitKey(1);
}


void OccupancyGrid::saveCostMapAsImage(const std::string& filename) const {
    cv::Mat image(height_, width_, CV_8UC1);

   // float max_cost = 1.0f;  // For scaling

float max_cost = 0.0f;
for (float val : cost_map_) {
    if (!std::isinf(val)) max_cost = std::max(max_cost, val);
}



    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            float cost = cost_map_[y * width_ + x];
            if (std::isinf(cost)) {
                image.at<uchar>(y, x) = 0; // Black for obstacles/infinity
            } else {
                //uchar value = static_cast<uchar>(255 * (1.0f - std::min(cost / max_cost, 1.0f)));
                uchar value = static_cast<uchar>(255 * (1.0f - cost / max_cost));
                image.at<uchar>(y, x) = value;
            }
        }
    }

    // Optional: Add coordinate labels (same as in showCostMap)
    for (int y = 0; y < height_; y += 20) {
        for (int x = 0; x < width_; x += 20) {
            //std::string label = std::to_string(x) + "," + std::to_string(y);
            float wx = (x - origin_x_) * resolution_;
float wy = (y - origin_y_) * resolution_;
std::string label = "(" + std::to_string(wx) + "," + std::to_string(wy) + ")";

            
            cv::putText(image, label, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.3, 100, 1);
        }
    }

    cv::imwrite(filename, image);
    std::cout << "[Info] Cost map saved as image: " << filename << std::endl;
}



bool OccupancyGrid::isOccupied(int x, int y) const {
    if (!isInside(x, y)) return false;
    return getLogOdds(x, y) > 1.0f;  // Tighter check
}


// --- Mouse Callback ---
static void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        auto* grid = static_cast<OccupancyGrid*>(userdata);
        float wx = (x - grid->getOriginX()) * grid->getResolution();
        float wy = (y - grid->getOriginY()) * grid->getResolution();
        std::cout << "[Click] Grid: (" << x << "," << y << ")  →  World: (" 
                  << wx << " m, " << wy << " m)" << std::endl;
    }
}

void OccupancyGrid::showCostMapWithClick() {
    cv::Mat image(height_, width_, CV_8UC1);

    float max_cost = 1.0f; // For visualization scaling

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            float cost = cost_map_[y * width_ + x];
            if (std::isinf(cost)) {
                image.at<uchar>(y, x) = 0; // Black
            } else {
                uchar value = static_cast<uchar>(255 * (1.0f - std::min(cost / max_cost, 1.0f)));
                image.at<uchar>(y, x) = value;
            }
        }
    }

    // Add grid labels (optional)
    for (int y = 0; y < height_; y += 20) {
        for (int x = 0; x < width_; x += 20) {
            float wx = (x - origin_x_) * resolution_;
            float wy = (y - origin_y_) * resolution_;
            std::string label = "(" + std::to_string(wx) + "," + std::to_string(wy) + ")";
            cv::putText(image, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 0.4, 100, 1);
        }
    }

    // Set up window and mouse callback
    cv::namedWindow("Clickable Cost Map", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Clickable Cost Map", onMouse, (void*)this);

    cv::imshow("Clickable Cost Map", image);
    std::cout << "[Info] Click on map to get goal coordinates.\n";
    cv::waitKey(0); // Wait until key press
}




static void onMouseSetGoal(int event, int x, int y, int, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        auto* grid = static_cast<OccupancyGrid*>(userdata);
        float wx = (x - grid->getOriginX()) * grid->getResolution();
        float wy = (y - grid->getOriginY()) * grid->getResolution();
        grid->goal_world_x_ = wx;
        grid->goal_world_y_ = wy;
        grid->goal_selected_ = true;
        std::cout << "[Goal Selected] World: (" << wx << " m, " << wy << " m)" << std::endl;
    }
}



void OccupancyGrid::showCostMapWithGoalSelection() {
    cv::Mat image(height_, width_, CV_8UC3);

    float max_cost = 1.0f;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            float cost = cost_map_[y * width_ + x];
            uchar val = std::isinf(cost) ? 0 : static_cast<uchar>(255 * (1.0f - std::min(cost / max_cost, 1.0f)));
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(val, val, val);
        }
    }

    drawGoal(image); // Draw if already selected

    cv::namedWindow("Select Goal on Map", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Select Goal on Map", onMouseSetGoal, (void*)this);
    cv::imshow("Select Goal on Map", image);
    std::cout << "[Click] Select a goal position on the map.\n";
    cv::waitKey(0);

    // Redraw with goal
    if (goal_selected_) {
        drawGoal(image);
        cv::imshow("Goal Marked", image);
        cv::imwrite("goal_cost_map.png", image);
        std::cout << "[Info] Goal marked and saved as goal_cost_map.png\n";
        cv::waitKey(0);
    }
}


void OccupancyGrid::drawGoal(cv::Mat& image) const {
    if (!goal_selected_) return;

    int gx = static_cast<int>(goal_world_x_ / resolution_) + origin_x_;
    int gy = static_cast<int>(goal_world_y_ / resolution_) + origin_y_;
    if (isInside(gx, gy)) {
        cv::circle(image, cv::Point(gx, gy), 4, cv::Scalar(0, 0, 255), -1);
    }
}



std::pair<float, float> OccupancyGrid::getSelectedGoal() const {
    return {goal_world_x_, goal_world_y_};
}

bool OccupancyGrid::isGoalSelected() const {
    return goal_selected_;
}



