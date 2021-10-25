// Copyright 2021 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "camera.hpp"

#include "filters/kalman_filter.hpp"
#include "filters/interacting_multiple_model_filter.hpp"
#include "types/models.hpp"

Camera::Camera(
  std::shared_ptr<ModelInputGenerator> model_input_generator,
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator)
{
  setup_ball_interacting_multiple_model_filter(
    model_input_generator,
    transmission_probability_generator);
  setup_robot_interacting_multiple_model_filter(
    model_input_generator,
    transmission_probability_generator);
}

void Camera::update(const CameraMeasurement & camera_measurement)
{
  for (size_t robot_id = 0; robot_id < 16; robot_id++) {
    if (!camera_measurement.yellow_robots.at(robot_id).empty()) {
      std::vector<Eigen::VectorXd> vectored_measurements =
        robot_measurements_to_vector(camera_measurement.yellow_robots.at(robot_id));
      yellow_team.at(robot_id).update(vectored_measurements);
    }

    if (!camera_measurement.blue_robots.at(robot_id).empty()) {
      std::vector<Eigen::VectorXd> vectored_measurements =
        robot_measurements_to_vector(camera_measurement.blue_robots.at(robot_id));
      blue_team.at(robot_id).update(vectored_measurements);
    }
  }

  if (!camera_measurement.ball.empty()) {
    std::vector<Eigen::VectorXd> vectored_measurements =
      ball_measurements_to_vector(camera_measurement.ball);
    ball.update(vectored_measurements);
  }
}

void Camera::predict()
{
  for (auto & yellow_robot : yellow_team) {
    yellow_robot.predict();
  }

  for (auto & blue_robot : blue_team) {
    blue_robot.predict();
  }

  ball.predict();
}

std::optional<Camera::BallWithScore> Camera::get_ball_estimate_with_score()
{
  // TODO: Return from the tracker
  return std::nullopt;
}

std::array<std::optional<Camera::RobotWithScore>,
  16> Camera::get_yellow_robot_estimates_with_score()
{
  // TODO: Return from the tracker
}

std::array<std::optional<Camera::RobotWithScore>, 16> Camera::get_blue_robot_estimates_with_score()
{
  // TODO: Return from the tracker
}

void Camera::setup_ball_interacting_multiple_model_filter(
  std::shared_ptr<ModelInputGenerator> model_input_generator,
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator)
{
  KalmanFilter base_kf_model;
  base_kf_model.set_F(Models::Ball::F);
  base_kf_model.set_B(Models::Ball::B);
  base_kf_model.set_H(Models::Ball::H);
  base_kf_model.set_Q(Models::Ball::Q);
  base_kf_model.set_R(Models::Ball::R);

  std::vector<Models::ModelType> model_types;
  model_types.emplace_back(Models::ModelType::BALL_ROLLING_FRICTION);
  model_types.emplace_back(Models::ModelType::BALL_SLIDING_FRICTION);
  model_types.emplace_back(Models::ModelType::BALL_BOUNCE_ON_ROBOT);
  model_types.emplace_back(Models::ModelType::BALL_STOP_ON_DRIBBLER);
  model_types.emplace_back(Models::ModelType::BALL_SLOW_KICK);
  model_types.emplace_back(Models::ModelType::BALL_MEDIUM_KICK);
  model_types.emplace_back(Models::ModelType::BALL_FAST_KICK);

  InteractingMultipleModelFilter base_track;
  base_track.setup(
    base_kf_model, model_types, model_input_generator, transmission_probability_generator);

  ball.set_base_track(base_track);
}

void Camera::setup_robot_interacting_multiple_model_filter(
  std::shared_ptr<ModelInputGenerator> model_input_generator,
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator)
{
  KalmanFilter base_kf_model;
  base_kf_model.set_F(Models::Robot::F);
  base_kf_model.set_B(Models::Robot::B);
  base_kf_model.set_H(Models::Robot::H);
  base_kf_model.set_Q(Models::Robot::Q);
  base_kf_model.set_R(Models::Robot::R);

  std::vector<Models::ModelType> model_types;
  model_types.emplace_back(Models::ModelType::ROBOT_NO_ACCEL);
  model_types.emplace_back(Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL);
  model_types.emplace_back(Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL);

  InteractingMultipleModelFilter base_track;
  base_track.setup(
    base_kf_model, model_types, model_input_generator,
    transmission_probability_generator);

  for (auto & yellow_robot : yellow_team) {
    yellow_robot.set_base_track(base_track);
  }

  for (auto & blue_robot : blue_team) {
    blue_robot.set_base_track(base_track);
  }
}

std::vector<Eigen::VectorXd> Camera::robot_measurements_to_vector(
  const std::vector<RobotMeasurement> & robot_measurements)
{
  std::vector<Eigen::VectorXd> vectored_measurements;
  vectored_measurements.reserve(robot_measurements.size());

  std::transform(
    robot_measurements.begin(), robot_measurements.end(),
    vectored_measurements.begin(),
    [](const RobotMeasurement & original) {
      return Eigen::Vector3d{original.position.x(), original.position.y(), original.theta};
    });

  return vectored_measurements;
}

std::vector<Eigen::VectorXd> Camera::ball_measurements_to_vector(
  const std::vector<BallMeasurement> & ball_measurements)
{
  std::vector<Eigen::VectorXd> vectored_measurements;
  vectored_measurements.reserve(ball_measurements.size());

  std::transform(
    ball_measurements.begin(), ball_measurements.end(),
    vectored_measurements.begin(),
    [](const BallMeasurement & original) {
      return original.position;
    });

  return vectored_measurements;
}
