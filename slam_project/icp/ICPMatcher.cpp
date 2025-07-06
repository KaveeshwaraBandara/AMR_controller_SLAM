#include "runICP.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <iostream>
#include <algorithm>

using namespace cv;
using namespace std;

bool is_converge(const Mat& Tr, float scale) {
    float delta_angle = 0.0001f;
    float delta_scale = scale * 0.0001f;

    return abs(1.0 - Tr.at<double>(0, 0)) < delta_angle &&
           abs(1.0 - Tr.at<double>(1, 1)) < delta_angle &&
           abs(Tr.at<double>(0, 1)) < delta_angle &&
           abs(Tr.at<double>(1, 0)) < delta_angle &&
           abs(Tr.at<double>(0, 2)) < delta_scale &&
           abs(Tr.at<double>(1, 2)) < delta_scale;
}

Mat applyTransform(const Mat& points, const Mat& Tr) {
    int N = points.rows;
    Mat points_homogeneous(N, 3, CV_64F);
    for (int i = 0; i < N; ++i) {
        points_homogeneous.at<double>(i, 0) = points.at<float>(i, 0);
        points_homogeneous.at<double>(i, 1) = points.at<float>(i, 1);
        points_homogeneous.at<double>(i, 2) = 1.0;
    }

    Mat result = points_homogeneous * Tr.t();
    return result(cv::Rect(0, 0, 2, result.rows)).clone();  // x, y only
}

Eigen::Matrix3f runICP(const std::vector<Eigen::Vector2f>& ref,
                       const std::vector<Eigen::Vector2f>& target) {
    if (ref.empty() || target.empty()) {
        std::cerr << "[ICP] Empty input point sets.\n";
        return Eigen::Matrix3f::Identity();
    }

    // Convert Eigen to OpenCV format
    Mat src(ref.size(), 2, CV_32F), dst(target.size(), 2, CV_32F);
    for (size_t i = 0; i < ref.size(); ++i) {
        src.at<float>(i, 0) = ref[i].x();
        src.at<float>(i, 1) = ref[i].y();
    }
    for (size_t i = 0; i < target.size(); ++i) {
        dst.at<float>(i, 0) = target[i].x();
        dst.at<float>(i, 1) = target[i].y();
    }

    Ptr<ml::KNearest> knn = ml::KNearest::create();
    Mat responses(src.rows, 1, CV_32F);
    for (int i = 0; i < src.rows; ++i)
        responses.at<float>(i) = static_cast<float>(i);
    knn->train(src, ml::ROW_SAMPLE, responses);

    Mat Tr = Mat::eye(3, 3, CV_64F);
    Mat dst_transformed = applyTransform(dst, Tr);

    float min_x = ref[0].x(), max_x = ref[0].x(), min_y = ref[0].y(), max_y = ref[0].y();
    for (const auto& pt : ref) {
        min_x = std::min(min_x, pt.x());
        max_x = std::max(max_x, pt.x());
        min_y = std::min(min_y, pt.y());
        max_y = std::max(max_y, pt.y());
    }
    float scale = std::max(max_x - min_x, max_y - min_y);

    for (int iter = 0; iter < 100; ++iter) {
        Mat results, neighborResponses, dist;
        knn->findNearest(dst_transformed, 1, results, neighborResponses, dist);

        Mat indices;
        results.convertTo(indices, CV_32S);

        double max_dist;
        minMaxIdx(dist, nullptr, &max_dist);
        double thresh = 0.8 * max_dist;

        vector<Point2f> filtered_dst, filtered_src;
        for (int j = 0; j < dist.rows; ++j) {
            if (dist.at<float>(j) < thresh) {
                filtered_dst.emplace_back(dst_transformed.at<double>(j, 0),
                                          dst_transformed.at<double>(j, 1));
                int idx = indices.at<int>(j);
                filtered_src.emplace_back(src.at<float>(idx, 0), src.at<float>(idx, 1));
            }
        }

        if (filtered_dst.empty()) break;

        Mat T = estimateAffinePartial2D(filtered_dst, filtered_src);
        if (T.empty()) break;

        // Promote to 3x3
        Mat T_homog = Mat::eye(3, 3, CV_64F);
        T.row(0).copyTo(T_homog.row(0).colRange(0, 3));
        T.row(1).copyTo(T_homog.row(1).colRange(0, 3));

        dst_transformed = applyTransform(dst, T_homog * Tr);
        Tr = T_homog * Tr;

        if (is_converge(T, scale)) break;
    }

    // Convert to Eigen::Matrix3f
    Eigen::Matrix3f result;
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 3; ++c)
            result(r, c) = static_cast<float>(Tr.at<double>(r, c));
    result(2, 0) = 0.0f;
    result(2, 1) = 0.0f;
    result(2, 2) = 1.0f;

    return result;
}

