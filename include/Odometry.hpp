#pragma once

struct Pose {
    float x;
    float y;
    float theta; // in radians
};

Pose updateOdometry(Pose prev, float delta_left, float delta_right, float wheel_base);
Pose updateWithIMU(Pose prev, float angular_velocity, float dt);

