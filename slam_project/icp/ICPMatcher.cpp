#include "ICPMatcher.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace cv;
using namespace std;

bool ICPMatcher::isConverged(const Mat& Tr, float scale) {
    float delta = 0.0001f;
    float move_thresh = scale * delta;

    return (std::abs(Tr.at<double>(0, 0) - 1) < delta &&
            std::abs(Tr.at<double>(1, 1) - 1) < delta &&
            std::abs(Tr.at<double>(0, 1)) < delta &&
            std::abs(Tr.at<double>(1, 0)) < delta &&
            std::abs(Tr.at<double>(0, 2)) < move_thresh &&
            std::abs(Tr.at<double>(1, 2)) < move_thresh);
}

cv::Mat ICPMatcher::runICP(const vector<Point2f>& d1, const vector<Point2f>& d2) {
    if (d1.size() < 5 || d2.size() < 5) {
        cerr << "ICP: Not enough points to compute transform.\n";
        return Mat::eye(2, 3, CV_64F);
    }

    Mat src(d1), dst(d2);
    src = src.reshape(1);
    dst = dst.reshape(1);

    Ptr<ml::KNearest> knn = ml::KNearest::create();
    Mat responses(src.rows, 1, CV_32F);
    for (int i = 0; i < src.rows; ++i) responses.at<float>(i, 0) = float(i);
    knn->train(src, ml::ROW_SAMPLE, responses);

    Mat Tr = Mat::eye(3, 3, CV_64F);
    Mat dst_transformed = dst.clone();

    auto [min_x, max_x] = minmax_element(d1.begin(), d1.end(), [](auto& a, auto& b){ return a.x < b.x; });
    auto [min_y, max_y] = minmax_element(d1.begin(), d1.end(), [](auto& a, auto& b){ return a.y < b.y; });
    float scale = std::max(max_x->x - min_x->x, max_y->y - min_y->y);

    for (int iter = 0; iter < 50; ++iter) {
        Mat results, dist;
        knn->findNearest(dst_transformed, 1, results, noArray(), dist);
        results.convertTo(results, CV_32S);

        double max_dist;
        minMaxLoc(dist, nullptr, &max_dist);
        double thresh = 0.8 * max_dist;

        vector<Point2f> src_filtered, dst_filtered;
        for (int i = 0; i < dist.rows; ++i) {
            if (dist.at<float>(i) < thresh) {
                dst_filtered.push_back(dst_transformed.at<Point2f>(i));
                src_filtered.push_back(d1[results.at<int>(i)]);
            }
        }

        if (src_filtered.size() < 5) break;

        Mat T = estimateAffinePartial2D(dst_filtered, src_filtered);
        if (T.empty()) break;

        // Ensure 2x3 format for transform
        try {
            transform(dst, dst_transformed, T);
        } catch (cv::Exception& e) {
            cerr << "ICP transform exception: " << e.what() << endl;
            return Mat::eye(2, 3, CV_64F);
        }

        Mat T_h = Mat::eye(3, 3, CV_64F);
        T.row(0).copyTo(T_h.row(0).colRange(0, 3));
        T.row(1).copyTo(T_h.row(1).colRange(0, 3));
        Tr = T_h * Tr;

        if (isConverged(T_h, scale)) break;
    }

    return Tr(Rect(0, 0, 3, 2)).clone();  // Return 2x3 final transformation
}

