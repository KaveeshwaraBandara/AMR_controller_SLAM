#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp> // Add this

struct Point2D {
    float x;
    float y;
};

std::vector<Point2D> latestPoints;

void OnLaserScanMsg(const gz::msgs::LaserScan &msg)
{
    latestPoints.clear();

    float angle = msg.angle_min();
    float angle_increment = msg.angle_step();

    for (int i = 0; i < msg.ranges_size(); ++i)
    {
        float range = msg.ranges(i);
        if (range > msg.range_min() && range < msg.range_max())
        {
            // Polar to Cartesian
            float x = range * cos(angle);
            float y = range * sin(angle);

            latestPoints.push_back({x, y});
        }
        angle += angle_increment;
    }

    std::cout << "Received " << latestPoints.size() << " points." << std::endl;
}

int main(int argc, char **argv)
{
    gz::transport::Node node;

    std::string topic = "/world/world_demo/model/tugbot/link/scan_front/sensor/scan_front/scan";

    if (!node.Subscribe(topic, &OnLaserScanMsg))
    {
        std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
        return -1;
    }

    std::cout << "Listening for LaserScan data on " << topic << " ..." << std::endl;

while (true)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Draw current point cloud
    cv::Mat map(500, 500, CV_8UC3, cv::Scalar(0,0,0));

    for (const auto& point : latestPoints)
    {
        int x = static_cast<int>(point.x * 50) + 250; // Scale + center
        int y = static_cast<int>(point.y * 50) + 250; // Scale + center

        if (x >= 0 && x < 500 && y >= 0 && y < 500)
            cv::circle(map, cv::Point(x, y), 1, cv::Scalar(0,255,0), -1);
    }

    cv::imshow("2D Lidar Map", map);
    cv::waitKey(1);
}
    return 0;
}

