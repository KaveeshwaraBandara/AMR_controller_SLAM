#include "Odometry.hpp"
#include <cmath>

Pose updateOdometry(Pose prev, float delta_left, float delta_right, float wheel_base) {
    float d = (delta_left + delta_right) / 2.0f;
    float delta_theta = (delta_right - delta_left) / wheel_base;

    Pose next;
    next.theta = prev.theta + delta_theta;
    next.x = prev.x + d * cos(next.theta);
    next.y = prev.y + d * sin(next.theta);
    return next;
}

Pose updateWithIMU(Pose prev, float angular_velocity, float dt) {
    Pose next = prev;
    next.theta += angular_velocity * dt;
    return next;
}

