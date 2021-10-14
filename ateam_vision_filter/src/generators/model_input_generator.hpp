// Given a ball location
// output the most likely kick direction
// as well as the most likely bounce direction

#include "types/robot.hpp"
#include "types/models.hpp"

#include <Eigen/Dense>

class ModelInputGenerator {
public:
    void update(const std::array<Robot, 16> & blue_robots,
                const std::array<Robot, 16> & yellow_robots);

    Eigen::VectorXd get_model_input(const Eigen::VectorXd & possible_ball,
                                    Models::Ball::ModelType model_type) const {}
private:
};