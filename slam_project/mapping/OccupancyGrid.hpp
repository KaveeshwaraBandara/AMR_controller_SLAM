#ifndef OCCUPANCY_GRID_HPP
#define OCCUPANCY_GRID_HPP

#include <vector>
#include <utility>  // for std::pair
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>


/*struct Pose2D {
    float x = 0.0f, y = 0.0f, theta = 0.0f;
};*/

struct Pose2D {
    float x, y, theta;
    Eigen::Matrix3f covariance; // EKF covariance at this pose
};


class OccupancyGrid {
public:
    OccupancyGrid(int width, int height, float resolution, float origin_x = 0.0f, float origin_y = 0.0f);
    float getLogOdds(int x, int y) const;
    void updateWithScan(const std::vector<std::pair<float, float>>& scan);
    void updateWithGlobalPoints(const std::vector<std::pair<float, float>>& points);
    void saveAsImage(const std::string& filename);
   // void saveAsImageWithPath(const std::string& filename, const std::vector<cv::Point2f>& path);
//   void saveAsImageWithTrajectory(const std::string& filename, const std::vector<std::pair<float, float>>& trajectory);
   //void saveAsImageWithTrajectory(const std::string& filename, const std::vector<Pose2D>& trajectory);
   void saveAsImageWithTrajectory(const std::string& filename, const std::vector<Pose2D>& trajectory);
   void showLiveMap(const std::vector<Pose2D>& trajectory);
   void drawCovarianceEllipse(cv::Mat& img, int cx, int cy, const Eigen::Matrix2f& cov, const cv::Scalar& color);
    void updateCostMap(float robot_radius);
    const std::vector<float>& getCostMap() const;
    float getCost(int x, int y) const;
    void showCostMap() const;
    float getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    bool isCellInside(int x, int y) const { return isInside(x, y); }
    bool isInside(int x, int y) const;
    
    bool isFree(int x, int y) const {
    if (!isInside(x, y)) return false;
    float log_odds = getLogOdds(x, y);
    return log_odds < -1.0f;  // Free if log_odds < -1.0
    }




private:
    int width_, height_;
    float resolution_; // meters per cell
    int origin_x_, origin_y_; // grid index of robot position (typically center)

    std::vector<float> log_odds_grid_;
    float log_odds_hit_ = 0.85f;
    float log_odds_miss_ = -0.4f;
    float log_odds_min_ = -4.0f;
    float log_odds_max_ = 4.0f;

    void setLogOdds(int x, int y, float delta);
    
    

    void raycastAndUpdate(int x0, int y0, int x1, int y1);
    
    std::vector<float> cost_map_;
    float robot_radius_cells_ = 0.0f;
    
    void inflateObstacles(float radius_cells);
    
};

#endif

