#pragma once

#include <Eigen/Dense>

class RobotMeasurement {
public:
    Eigen::Vector2d position;
    double theta;
};