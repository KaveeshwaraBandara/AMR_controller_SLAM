#ifndef OCCUPANCY_GRID_HPP
#define OCCUPANCY_GRID_HPP

#include <vector>
#include <utility>  // for std::pair
#include <string>

struct Pose2D {
    float x = 0.0f, y = 0.0f, theta = 0.0f;
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
   void saveAsImageWithTrajectory(const std::string& filename, const std::vector<Pose2D>& trajectory);
   void showLiveMap(const std::vector<Pose2D>& trajectory);





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
    
    bool isInside(int x, int y) const;

    void raycastAndUpdate(int x0, int y0, int x1, int y1);
};

#endif

