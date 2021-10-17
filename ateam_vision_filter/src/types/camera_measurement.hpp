#pragma once

#include "types/ball_measurement.hpp"
#include "types/robot_measurement.hpp"

#include <array>
#include <vector>

struct CameraMeasurement {
    std::array<std::vector<RobotMeasurement>, 16> yellow_robots;
    std::array<std::vector<RobotMeasurement>, 16> blue_robots;
    std::vector<BallMeasurement> ball;
};