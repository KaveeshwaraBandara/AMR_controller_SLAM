#pragma once

#include <string>
#include <vector>
#include <utility>
#include "OccupancyGrid.hpp"

class TrajectoryVisualizer {
public:
    // Draws the trajectory over the given map and saves to output file
    static void drawTrajectoryOnMap(const std::string& map_path,
                                    const std::vector<Pose2D>& trajectory,
                                    const OccupancyGrid& grid,
                                    const std::pair<int, int>& goal_grid,
                                    const std::string& output_path = "map_with_trajectory.png");
};


