#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <opencv2/opencv.hpp>

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

cv::Mat runICP(const std::vector<cv::Point2f>& d1, const std::vector<cv::Point2f>& d2)
 {
    // Convert data to Mat
    Mat src(d1.size(), 2, CV_32F);
    for (size_t i = 0; i < d1.size(); ++i) {
        src.at<float>(i, 0) = d1[i].x;
        src.at<float>(i, 1) = d1[i].y;
    }
    
    Mat dst(d2.size(), 2, CV_32F);
    for (size_t i = 0; i < d2.size(); ++i) {
        dst.at<float>(i, 0) = d2[i].x;
        dst.at<float>(i, 1) = d2[i].y;
    }
    
    // Create KNN model
    Ptr<ml::KNearest> knn = ml::KNearest::create();
    
    // Responses (indices)
    Mat responses(src.rows, 1, CV_32F);
    for (int i = 0; i < src.rows; ++i) {
        responses.at<float>(i) = static_cast<float>(i);
    }
    
    // Train the model
    knn->train(src, ml::ROW_SAMPLE, responses);
    
    // Initial transformation (identity matrix)
    Mat Tr = (Mat_<double>(3, 3) << 1, 0, 0,
                                     0, 1, 0,
                                     0, 0, 1);
    
    // Apply initial transform
    Mat dst_transformed;
    transform(dst.reshape(2, 1), dst_transformed, Tr(Rect(0, 0, 3, 2)));
    dst_transformed = dst_transformed.reshape(1, dst.rows);
    
    // Calculate scale for convergence checking
    auto [min_x, max_x] = minmax_element(d1.begin(), d1.end(), 
        [](const Point2f& a, const Point2f& b) { return a.x < b.x; });
    auto [min_y, max_y] = minmax_element(d1.begin(), d1.end(), 
        [](const Point2f& a, const Point2f& b) { return a.y < b.y; });
    float scale_x = max_x->x - min_x->x;
    float scale_y = max_y->y - min_y->y;
    float scale = max(scale_x, scale_y);
    
    for (int i = 0; i < 100; ++i) {
        // Find nearest neighbors
        Mat results, neighborResponses, dist;
        knn->findNearest(dst_transformed, 1, results, neighborResponses, dist);
        
        // Convert results to indices
        Mat indices;
        results.convertTo(indices, CV_32S);
        
        // Filter outliers (keep 80% closest points)
        double thresh;
        minMaxIdx(dist, nullptr, &thresh, nullptr, nullptr);
        thresh = 0.8 * thresh; // Keep 80% closest points
        
        vector<Point2f> filtered_dst, filtered_src;
        for (int j = 0; j < dist.rows; ++j) {
            if (dist.at<float>(j) < thresh) {
                filtered_dst.push_back(Point2f(dst_transformed.at<float>(j, 0), 
                                              dst_transformed.at<float>(j, 1)));
                int idx = indices.at<int>(j);
                filtered_src.push_back(d1[idx]);
            }
        }
        
        if (filtered_dst.empty()) {
            break;
        }
        
        // Estimate rigid transform
        Mat T = estimateAffinePartial2D(filtered_dst, filtered_src);
        if (T.empty()) {
            break;
        }
        
        // Apply transformation
        transform(dst_transformed.reshape(2, 1), dst_transformed, T);
        dst_transformed = dst_transformed.reshape(1, dst.rows);
        
        // Update total transformation
        Mat T_homog = Mat::zeros(3, 3, CV_64F);
        T.row(0).copyTo(T_homog.row(0));
        T.row(1).copyTo(T_homog.row(1));
        T_homog.at<double>(2, 2) = 1.0;
        Tr = T_homog * Tr;
        
        // Check convergence
        if (is_converge(T, scale)) {
            break;
        }
    }
    
    return Tr(Rect(0, 0, 3, 2)).clone();
}

