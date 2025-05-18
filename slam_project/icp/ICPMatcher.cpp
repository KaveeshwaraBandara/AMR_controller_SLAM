#include "ICPMatcher.hpp"
#include <opencv2/ml.hpp>
#include <limits>
#include <cmath>

using namespace cv;
using namespace std;

static bool is_converge(const Mat& Tr, float scale) {
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

cv::Mat ICPMatcher::align(const vector<Point2f>& d1, const vector<Point2f>& d2, int max_iterate) {
    Mat src(d1.size(), 2, CV_32F), dst(d2.size(), 2, CV_32F);
    for (size_t i = 0; i < d1.size(); ++i) {
        src.at<float>(i, 0) = d1[i].x;
        src.at<float>(i, 1) = d1[i].y;
    }
    for (size_t i = 0; i < d2.size(); ++i) {
        dst.at<float>(i, 0) = d2[i].x;
        dst.at<float>(i, 1) = d2[i].y;
    }

    Ptr<ml::KNearest> knn = ml::KNearest::create();
    knn->train(src, ml::ROW_SAMPLE, Mat::eye(src.rows, 1, CV_32F));

    Mat Tr = Mat::eye(3, 3, CV_64F);
    Mat dst_transformed;
    cv::transform(dst.reshape(2, 1), dst_transformed, Tr(Rect(0, 0, 3, 2)));
    dst_transformed = dst_transformed.reshape(1, dst.rows);

    float scale = 1.0f;
    for (int i = 0; i < max_iterate; ++i) {
        Mat results, neighborResponses, dist;
        knn->findNearest(dst_transformed, 1, results, neighborResponses, dist);

        Mat indices;
        results.convertTo(indices, CV_32S);

        double max_dist;
        minMaxIdx(dist, nullptr, &max_dist, nullptr, nullptr);
        double thresh = 0.8 * max_dist;

        vector<Point2f> filtered_dst, filtered_src;
        for (int j = 0; j < dist.rows; ++j) {
            if (dist.at<float>(j) < thresh) {
                filtered_dst.emplace_back(dst_transformed.at<float>(j, 0), dst_transformed.at<float>(j, 1));
                int idx = indices.at<int>(j);
                filtered_src.push_back(d1[idx]);
            }
        }

        if (filtered_dst.empty()) break;

        Mat T = estimateAffinePartial2D(filtered_dst, filtered_src);
        if (T.empty()) break;

        transform(dst_transformed.reshape(2, 1), dst_transformed, T);
        dst_transformed = dst_transformed.reshape(1, dst.rows);

        Mat T_homog = Mat::eye(3, 3, CV_64F);
        T.row(0).copyTo(T_homog.row(0));
        T.row(1).copyTo(T_homog.row(1));
        Tr = T_homog * Tr;

        if (is_converge(T, scale)) break;
    }

    return Tr(Rect(0, 0, 3, 2)).clone();
}

