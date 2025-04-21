#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

struct LidarReading {
    float angle_deg;
    float distance_m;
};

const int MAP_SIZE = 800;
const float MAP_RESOLUTION = 0.05f; // meters per cell
const float PROB_HIT = 0.9f;
const float PROB_MISS = 0.4f;
const float PROB_PRIOR = 0.5f;

const float MIN_LOG_ODDS = log(PROB_MISS / (1 - PROB_MISS));
const float MAX_LOG_ODDS = log(PROB_HIT / (1 - PROB_HIT));

float probToLogOdds(float p) {
    return log(p / (1.0f - p));
}

float logOddsToProb(float l) {
    return 1.0f - 1.0f / (1.0f + exp(l));
}

std::vector<LidarReading> loadLidarCSV(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<LidarReading> readings;
    std::string line;
    //std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string angleStr, distStr;
        if (std::getline(ss, angleStr, ',') && std::getline(ss, distStr)) {
            float angle = std::stof(angleStr);
            float dist = std::stof(distStr);
            readings.push_back({angle, dist});
        }
    }
    return readings;
}

void drawRobot(cv::Mat& img, cv::Point center, float orientation_rad) {
    cv::circle(img, center, 6, cv::Scalar(255, 0, 0), -1); // Robot center
    cv::line(img, center, center + cv::Point(20 * cos(orientation_rad), 20 * sin(orientation_rad)), cv::Scalar(255, 0, 0), 2); // Orientation
    // Draw axes
    cv::line(img, center, center + cv::Point(30, 0), cv::Scalar(0, 0, 255), 2); // X - Red
    cv::line(img, center, center + cv::Point(0, -30), cv::Scalar(0, 255, 0), 2); // Y - Green
}

void updateOccupancyGrid(cv::Mat& log_odds, const std::vector<LidarReading>& readings, cv::Point2f robot_pos, float orientation_rad) {
    for (const auto& reading : readings) {
        float angle_rad = reading.angle_deg * CV_PI / 180.0f + orientation_rad;
        float x = robot_pos.x + reading.distance_m * cos(angle_rad);
        float y = robot_pos.y - reading.distance_m * sin(angle_rad);

        cv::Point cell_end((int)(x / MAP_RESOLUTION), (int)(y / MAP_RESOLUTION));
        cv::Point cell_start((int)(robot_pos.x / MAP_RESOLUTION), (int)(robot_pos.y / MAP_RESOLUTION));

        cv::LineIterator it(log_odds, cell_start, cell_end);
        for (int i = 0; i < it.count; i++, ++it) {
            float& cell = log_odds.at<float>(it.pos());
            if (i < it.count - 1) {
                cell = std::clamp(cell + probToLogOdds(PROB_MISS), MIN_LOG_ODDS, MAX_LOG_ODDS);
            } else {
                cell = std::clamp(cell + probToLogOdds(PROB_HIT), MIN_LOG_ODDS, MAX_LOG_ODDS);
            }
        }
    }
}

cv::Mat renderMap(const cv::Mat& log_odds) {
    cv::Mat map(MAP_SIZE, MAP_SIZE, CV_8UC1);
    for (int y = 0; y < MAP_SIZE; y++) {
        for (int x = 0; x < MAP_SIZE; x++) {
            float prob = logOddsToProb(log_odds.at<float>(y, x));
            map.at<uchar>(y, x) = static_cast<uchar>(prob * 255);
        }
    }
    return map;
}

int main() {
    std::vector<std::string> frames = {"../scan1.csv", "../scan2.csv", "../scan3.csv"};
    cv::Mat log_odds(MAP_SIZE, MAP_SIZE, CV_32F, cv::Scalar(probToLogOdds(PROB_PRIOR)));
    cv::Point2f robot_pos(0.0f, 20.0f); // meters
    float orientation = 0;

    for (const auto& csv : frames) {
        auto readings = loadLidarCSV(csv);
        updateOccupancyGrid(log_odds, readings, robot_pos, orientation);
        cv::Mat map_img = renderMap(log_odds);

        // Draw robot
        drawRobot(map_img, cv::Point((int)(robot_pos.x / MAP_RESOLUTION), (int)(robot_pos.y / MAP_RESOLUTION)), orientation);

        cv::imshow("Occupancy Grid", map_img);
        cv::waitKey(500);

        // Simulate robot movement
        robot_pos += cv::Point2f(0.5f, 0); // Move right 0.5m
        orientation += 0.01f;
    }

    cv::waitKey(0);
    return 0;
}

