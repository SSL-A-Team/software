#include "generators/transmission_probability_generator.hpp"

void TransmissionProbabilityGenerator::update(
  const std::array<Robot, 16> & blue_robots,
  const std::array<Robot, 16> & yellow_robots,
  const Ball & ball)
{
  this->blue_robots = blue_robots;
  this->yellow_robots = yellow_robots;
  this->ball = ball;
}

double TransmissionProbabilityGenerator::get_transmission_probability(
  const Eigen::VectorXd & possible_state,
  const Models::ModelType & from_model,
  const Models::ModelType & to_model) const
{
  // Ball
  
  // Const transition from Rolling to sliding
  // Increase bounce if ball is going at not the mouth of11 robot
  // Increase stop if ball is going at the mouth of robot
  // After ball is near mouth of robot increase kick probabilities
  // If prev was kick, increase rolling probability

  // Robot

  // Most likely to stay the same
  // Equally like to swap to other
}