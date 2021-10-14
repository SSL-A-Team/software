#include "generators/model_input_generator.hpp"

void ModelInputGenerator::update(
  const std::array<Robot, 16> & blue_robots,
  const std::array<Robot, 16> & yellow_robots,
  const Ball & ball)
{
  this->blue_robots = blue_robots;
  this->yellow_robots = yellow_robots;
  this->ball = ball;  
}

Eigen::VectorXd ModelInputGenerator::get_model_input(
  const Eigen::VectorXd & possible_state,
  const Models::ModelType & model_type) const
{
  // Ball
  if (model_type == Models::ModelType::BALL_ROLLING_FRICTION)
  {
    // Ret rolling friction deccell to 0
  }
  else if (model_type == Models::ModelType::BALL_SLIDING_FRICTION)
  {
    // Ret sliding friction deccell to 0
  }
  else if (model_type == Models::ModelType::BALL_BOUNCE_ON_ROBOT)
  {
    // Get closest robot
    // Figure out bounce (assuming robot is circle for now)
  }
  else if (model_type == Models::ModelType::BALL_STOP_ON_DRIBBLER)
  {
    // Negate current speed like ball instantly damps on dribbler
  }
  else if (model_type == Models::ModelType::BALL_SLOW_KICK)
  {
    // Facing direction of closest robot at 2 m/s
  }
  else if (model_type == Models::ModelType::BALL_MEDIUM_KICK)
  {
    // Facing direction of closest robot at 4 m/s
  }
  else if (model_type == Models::ModelType::BALL_FAST_KICK)
  {
    // Facing direction of closest robot at 6 m/s
  }

  // Robot
  if (model_type == Models::ModelType::ROBOT_NO_ACCEL)
  {
    //return Eigen::VectorXd{0, 0, 0, 0, 0, 0, 0, 0, 0};
  }
  else if (model_type == Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL)
  {
    // Accel at X m/s2 towards ball
  }
  else if (model_type == Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL)
  {
    // Accel at X m/s2 away from ball
  }
  
  // ERROR
  return 0 * possible_state;
}