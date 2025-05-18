#ifndef ICP_MATCHER_HPP
#define ICP_MATCHER_HPP

#include <vector>
#include <opencv2/opencv.hpp>

class ICPMatcher {
public:
    static cv::Mat align(const std::vector<cv::Point2f>& ref,
                         const std::vector<cv::Point2f>& src,
                         int max_iter = 100);
};

#endif

