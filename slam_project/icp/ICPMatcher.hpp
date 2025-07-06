#ifndef ICP_MATCHER_HPP
#define ICP_MATCHER_HPP

#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <vector>

class ICPMatcher {
public:
    // Perform ICP between two sets of 2D points
    cv::Mat runICP(const std::vector<cv::Point2f>& d1, const std::vector<cv::Point2f>& d2);

private:
    bool isConverged(const cv::Mat& T, float scale);
};

#endif

