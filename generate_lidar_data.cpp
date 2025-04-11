#include <iostream>
#include <fstream>
#include <cmath>
#include <random>

int main() {
    std::ofstream file("simulated_lidar.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open file\n";
        return 1;
    }

    file << "frame_id,angle_deg,distance_m\n";

    const int total_frames = 20;
    const int points_per_frame = 360;
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distance_dist(1.0f, 3.0f);

    for (int frame = 0; frame < total_frames; ++frame) {
        for (int angle = 0; angle < points_per_frame; ++angle) {
            float distance = distance_dist(generator);
            file << frame << "," << angle << "," << distance << "\n";
        }
    }

    file.close();
    std::cout << "CSV generated successfully!\n";
    return 0;
}
