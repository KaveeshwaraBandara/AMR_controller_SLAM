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


void OccupancyGrid::updateWithGlobalPoints(const std::vector<cv::Point2f>& points) {
    for (const auto& pt : points) {
        int gx = static_cast<int>(pt.x / resolution_) + origin_x_;
        int gy = static_cast<int>(pt.y / resolution_) + origin_y_;
        raycastAndUpdate(origin_x_, origin_y_, gx, gy);
        setLogOdds(gx, gy, log_odds_hit_);
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

void OccupancyGrid::saveAsImageWithPath(const std::string& filename, const std::vector<cv::Point2f>& path) {
    cv::Mat image(height_, width_, CV_8UC3);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            float val = getLogOdds(x, y);
            cv::Vec3b color;

            if (val > 1.0f)
                color = {0, 0, 0};          // occupied = black
            else if (val < -1.0f)
                color = {255, 255, 255};    // free = white
            else
                color = {128, 128, 128};    // unknown = gray

            image.at<cv::Vec3b>(y, x) = color;
        }
    }

    // Draw path
    for (const auto& pt : path) {
        int gx = static_cast<int>(pt.x / resolution_) + origin_x_;
        int gy = static_cast<int>(pt.y / resolution_) + origin_y_;

        if (isInside(gx, gy)) {
            cv::circle(image, cv::Point(gx, gy), 1, cv::Scalar(0, 0, 255), -1);  // red dot
        }
    }

    cv::imwrite(filename, image);
    std::cout << "[Info] Saved map with path: " << filename << std::endl;
}

