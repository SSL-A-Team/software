#pragma once

#include "types/ball.hpp"
#include "types/models.hpp"
#include "types/robot.hpp"

#include <Eigen/Dense>

#include <array>

class ModelInputGenerator {
public:
    void update(const std::array<Robot, 16> & blue_robots,
                const std::array<Robot, 16> & yellow_robots,
                const Ball & ball);

    Eigen::VectorXd get_model_input(const Eigen::VectorXd & possible_state,
                                    const Models::ModelType & model_type) const;
private:
    std::array<Robot, 16> blue_robots;
    std::array<Robot, 16> yellow_robots;
    Ball ball;
};