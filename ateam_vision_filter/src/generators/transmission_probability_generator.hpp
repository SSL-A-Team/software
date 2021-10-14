// Builds a transmission probability matrix
// given last robot estimate positions
// last ball estimate position
// Transmission of ball from slide to roll, etc

#include "../types/ball.hpp"
#include "../types/robot.hpp"
#include "../types/models.hpp"

class TransmissionProbabilityGenerator {
public:
    void update(const std::array<Robot, 16> & blue_robots,
                const std::array<Robot, 16> & yellow_robots);

    double get_tranmission_probability(const Ball & possible_ball,
                                       Models::Ball::ModelType from_model,
                                       Models::Ball::ModelType to_model) const {}

private:
};