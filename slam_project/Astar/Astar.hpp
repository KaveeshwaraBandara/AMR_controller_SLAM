#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <sstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <unistd.h>     // For usleep()
#include <fcntl.h>      // For file control (O_RDWR, O_NOCTTY, etc.)
#include <termios.h>    // For serial port settings
#include "OccupancyGrid.hpp" // Include the OccupancyGrid header

const double CELL_SIZE = 0.05; // 5cm per cell (must match OccupancyGrid resolution)
const double LINEAR_VELOCITY = 0.03; // 0.07 m/s

struct Node {
    int x, y;
    double g, h, f;
    Node* parent;

    Node(int x, int y, Node* parent = nullptr) : x(x), y(y), g(0), h(0), f(0), parent(parent) {}

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

std::string getKey(int x, int y);

bool isValid(int x, int y, const OccupancyGrid& grid);
double heuristic(int x1, int y1, int x2, int y2);
double heuristic(int x1, int y1, int x2, int y2);
double heuristic(int x1, int y1, int x2, int y2);
std::vector<Node> aStar(const OccupancyGrid& grid, int startX, int startY, int goalX, int goalY);
void visualizePath(const OccupancyGrid& grid, const std::vector<Node>& path, int startX, int startY, int goalX, int goalY);
void calculateVelocityCommands(const std::vector<Node>& path, double initial_orientation_deg);
void createTestEnvironment(OccupancyGrid& grid);
#endif
