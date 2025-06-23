#ifndef ICP_MATCHER_HPP
#define ICP_MATCHER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat runICP(const std::vector<cv::Point2f>& ref_scan, const std::vector<cv::Point2f>& cur_scan);

#endif

