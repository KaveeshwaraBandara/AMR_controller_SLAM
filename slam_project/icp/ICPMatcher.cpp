#include "ICPMatcher.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

using namespace cv;
using namespace std;

bool ICPMatcher::isConverged(const Mat& Tr, float scale) {
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

cv::Mat ICPMatcher::runICP(const std::vector<cv::Point2f>& d1, const std::vector<cv::Point2f>& d2) {
    if (d1.size() < 3 || d2.size() < 3) {
        cerr << "ICP: Not enough points to compute transform.\n";
        return Mat::eye(2, 3, CV_64F);
    }

    Mat src = Mat(d1).reshape(1);
    Mat dst = Mat(d2).reshape(1);

    Ptr<ml::KNearest> knn = ml::KNearest::create();
    Mat responses(src.rows, 1, CV_32F);
    for (int i = 0; i < src.rows; ++i)
        responses.at<float>(i, 0) = static_cast<float>(i);
    knn->train(src, ml::ROW_SAMPLE, responses);

    Mat Tr = Mat::eye(3, 3, CV_64F);
    Mat dst_transformed = dst.clone();

    auto [min_x, max_x] = minmax_element(d1.begin(), d1.end(), [](auto& a, auto& b) { return a.x < b.x; });
    auto [min_y, max_y] = minmax_element(d1.begin(), d1.end(), [](auto& a, auto& b) { return a.y < b.y; });
    float scale_x = max_x->x - min_x->x;
    float scale_y = max_y->y - min_y->y;
    float scale = std::max(scale_x, scale_y);

    for (int iter = 0; iter < 100; ++iter) {
        Mat results, dist;
        knn->findNearest(dst_transformed, 1, results, noArray(), dist);
        results.convertTo(results, CV_32S);

        double max_dist;
        minMaxLoc(dist, nullptr, &max_dist);
        double thresh = 0.8 * max_dist;

        std::vector<Point2f> filtered_src, filtered_dst;
        for (int i = 0; i < dist.rows; ++i) {
            if (dist.at<float>(i) < thresh) {
                filtered_dst.push_back(dst_transformed.at<Point2f>(i));
                int idx = results.at<int>(i);
                filtered_src.push_back(d1[idx]);
            }
        }

        if (filtered_dst.size() < 3) break;

        Mat T = estimateAffinePartial2D(filtered_dst, filtered_src);
        if (T.empty()) break;

        try {
            transform(dst, dst_transformed, T);
        } catch (const cv::Exception& e) {
            cerr << "ICP transform exception: " << e.what() << endl;
            break;
        }

        Mat T_h = Mat::eye(3, 3, CV_64F);
        T.row(0).copyTo(T_h.row(0).colRange(0, 3));
        T.row(1).copyTo(T_h.row(1).colRange(0, 3));
        Tr = T_h * Tr;

        if (isConverged(T_h, scale)) break;
    }

    return Tr(Rect(0, 0, 3, 2)).clone();
}

