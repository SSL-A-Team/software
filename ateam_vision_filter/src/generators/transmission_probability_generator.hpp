// Builds a transmission probability matrix
// given last robot estimate positions
// last ball estimate position
// Transmission of ball from slide to roll, etc

#include "types/ball.hpp"
#include "types/models.hpp"
#include "types/robot.hpp"

#include <Eigen/Dense>

#include <array>

class TransmissionProbabilityGenerator {
public:
    void update(const std::array<Robot, 16> & blue_robots,
                const std::array<Robot, 16> & yellow_robots,
                const Ball & ball);

    double get_tranmission_probability(const Eigen::VectorXd & possible_state,
                                       Models::ModelType from_model,
                                       Models::ModelType to_model) const {}

private:
    std::array<Robot, 16> blue_robots;
    std::array<Robot, 16> yellow_robots;
    Ball ball;
};