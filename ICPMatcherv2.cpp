#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace std;
using namespace cv;



bool is_converge(const Mat& Tr, float scale) {
    float delta_angle = 0.0001f;
    float delta_scale = scale * 0.0001f;
    
    float min_cos = 1 - delta_angle;
    float max_cos = 1 + delta_angle;
    float min_sin = -delta_angle;
    float max_sin = delta_angle;
    float min_move = -delta_scale;
    float max_move = delta_scale;
    
    return (min_cos < Tr.at<double>(0, 0) && Tr.at<double>(0, 0) < max_cos &&
            min_cos < Tr.at<double>(1, 1) && Tr.at<double>(1, 1) < max_cos &&
            min_sin < -Tr.at<double>(1, 0) && -Tr.at<double>(1, 0) < max_sin &&
            min_sin < Tr.at<double>(0, 1) && Tr.at<double>(0, 1) < max_sin &&
            min_move < Tr.at<double>(0, 2) && Tr.at<double>(0, 2) < max_move &&
            min_move < Tr.at<double>(1, 2) && Tr.at<double>(1, 2) < max_move);
}

cv::Mat runICP(const std::vector<cv::Point2f>& d1, const std::vector<cv::Point2f>& d2) {
    // Accurate SVD-based 2D point set alignment (Horn's method)
    using namespace Eigen;
    if (d1.size() != d2.size() || d1.empty()) return cv::Mat();

    MatrixX2f src(d1.size(), 2), dst(d2.size(), 2);
    for (size_t i = 0; i < d1.size(); ++i) {
        src(i, 0) = d1[i].x; src(i, 1) = d1[i].y;
        dst(i, 0) = d2[i].x; dst(i, 1) = d2[i].y;
    }

    Vector2f src_mean = src.colwise().mean();
    Vector2f dst_mean = dst.colwise().mean();
    MatrixX2f src_centered = src.rowwise() - src_mean.transpose();
    MatrixX2f dst_centered = dst.rowwise() - dst_mean.transpose();

    Matrix2f H = dst_centered.transpose() * src_centered;
    JacobiSVD<Matrix2f> svd(H, ComputeFullU | ComputeFullV);
    Matrix2f R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0) R.col(1) *= -1;
    Vector2f t = src_mean - R * dst_mean;

    cv::Mat T = (cv::Mat_<double>(2, 3) <<
        R(0,0), R(0,1), t(0),
        R(1,0), R(1,1), t(1));
    return T;
}

