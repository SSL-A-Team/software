#pragma once

#include <Eigen/Dense>

struct Ball {
    Eigen::Vector2d position;

    Eigen::Vector2d velocity;

    Eigen::Vector2d acceleration;
};