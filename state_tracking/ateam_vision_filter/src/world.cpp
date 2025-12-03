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

#include "world.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace ateam_vision_filter
{

World::World(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface)
: params_interface_(params_interface),
  model_input_generator(std::make_shared<ModelInputGenerator>()),
  transmission_probability_generator(std::make_shared<TransmissionProbabilityGenerator>()) {}

void World::update_camera(const CameraID & cameraID, const CameraMeasurement & measurement)
{
  // Add camera if it doesn't exist yet
  cameras.try_emplace(cameraID,
      Camera(model_input_generator, transmission_probability_generator, params_interface_));

  // Update the specific camera
  cameras.at(cameraID).update(measurement);
}
void World::predict(const int ignore_side)
{
  ignore_side_ = ignore_side;

  // First, we predict all the things
  for (auto & camera_pair : cameras) {
    camera_pair.second.set_ignored_half(ignore_side);
    camera_pair.second.predict();
  }

  auto ball_estimate = get_ball_estimate();
  auto yellow_robots_estimate = get_yellow_robots_estimate();
  auto blue_robots_estimate = get_blue_robots_estimate();
  model_input_generator->update(blue_robots_estimate, yellow_robots_estimate, ball_estimate);
  transmission_probability_generator->update(
    blue_robots_estimate, yellow_robots_estimate,
    ball_estimate);
}

std::optional<Ball> World::get_ball_estimate()
{
  using BallWithScore = std::pair<Ball, double>;
  std::vector<BallWithScore> balls_with_scores;

  // Grab estimates from cameras
  for (auto & camera_pair : cameras) {
    std::optional<BallWithScore> possible_ball_with_score =
      camera_pair.second.get_ball_estimate_with_score();


    if (possible_ball_with_score.has_value()) {
      balls_with_scores.emplace_back(possible_ball_with_score.value());
    }
  }

  // If we have no estimates, there is no ball
  if (balls_with_scores.empty()) {
    return std::nullopt;
  }

  Ball merged_ball;
  double total_score = 0.0;
  for (const auto & ball_with_score : balls_with_scores) {
    const auto & ball = ball_with_score.first;
    const auto & score = ball_with_score.second + 1;

    merged_ball.position += score * ball.position;
    merged_ball.velocity += score * ball.velocity;
    merged_ball.acceleration += score * ball.acceleration;
    total_score += score;
  }

  merged_ball.position /= total_score;
  merged_ball.velocity /= total_score;
  merged_ball.acceleration /= total_score;

  return merged_ball;
}

std::array<std::optional<Robot>, 16> World::get_yellow_robots_estimate()
{
  using RobotWithScore = std::pair<Robot, double>;
  std::array<std::vector<RobotWithScore>, 16> yellow_robots_with_scores;

  // Grab estimates from camera
  for (auto & camera_pair : cameras) {
    std::array<std::optional<RobotWithScore>, 16> possible_yellow_robots_with_score =
      camera_pair.second.get_yellow_robot_estimates_with_score();

    for (size_t yellow_id = 0; yellow_id < 16; yellow_id++) {
      if (possible_yellow_robots_with_score.at(yellow_id).has_value()) {
        yellow_robots_with_scores.at(yellow_id).emplace_back(
          possible_yellow_robots_with_score.at(yellow_id).value());
      }
    }
  }

  // Try to merge them together
  // return nullopt if there are no measurements
  std::array<std::optional<Robot>, 16> yellow_robots_estimates;
  for (size_t yellow_id = 0; yellow_id < 16; yellow_id++) {
    if (yellow_robots_with_scores.at(yellow_id).empty()) {
      yellow_robots_estimates.at(yellow_id) = std::nullopt;
    } else {
      // Merge robots based on weighted average of their scores
      yellow_robots_estimates.at(yellow_id) = Robot();

      auto & output_robot = yellow_robots_estimates.at(yellow_id).value();
      Eigen::Vector2d output_angle{0, 0};
      double total_score = 0.0;

      for (const auto & robot_with_score : yellow_robots_with_scores.at(yellow_id)) {
        const Robot & robot = std::get<0>(robot_with_score);
        const double & score = std::get<1>(robot_with_score) + 1;

        output_robot.position += score * robot.position;
        output_angle += score * Eigen::Vector2d{std::cos(robot.theta), std::sin(
            robot.theta)};
        output_robot.velocity += score * robot.velocity;
        output_robot.omega += score * robot.omega;
        output_robot.acceleration += score * robot.acceleration;
        output_robot.alpha += score * robot.alpha;

        total_score += score;
      }

      output_robot.position /= total_score;
      output_robot.theta = 0.0;
      if (!output_angle.isZero()) {
        output_robot.theta = std::atan2(output_angle.y(), output_angle.x());
      }
      output_robot.velocity /= total_score;
      output_robot.omega /= total_score;
      output_robot.acceleration /= total_score;
      output_robot.alpha /= total_score;
    }
  }

  return yellow_robots_estimates;
}

std::array<std::optional<Robot>, 16> World::get_blue_robots_estimate()
{
  using RobotWithScore = std::pair<Robot, double>;
  std::array<std::vector<RobotWithScore>, 16> blue_robots_with_scores;

  // Grab estimates from camera
  for (auto & camera_pair : cameras) {
    std::array<std::optional<RobotWithScore>, 16> possible_blue_robots_with_score =
      camera_pair.second.get_blue_robot_estimates_with_score();

    for (size_t blue_id = 0; blue_id < 16; blue_id++) {
      if (possible_blue_robots_with_score.at(blue_id).has_value()) {
        blue_robots_with_scores.at(blue_id).emplace_back(
          possible_blue_robots_with_score.at(blue_id).value());
      }
    }
  }

  // Try to merge them together
  // return nullopt if there are no measurements
  std::array<std::optional<Robot>, 16> blue_robots_estimates;
  for (size_t blue_id = 0; blue_id < 16; blue_id++) {
    if (blue_robots_with_scores.at(blue_id).empty()) {
      blue_robots_estimates.at(blue_id) = std::nullopt;
    } else {
      // Merge robots based on weighted average of their scores
      blue_robots_estimates.at(blue_id) = Robot();

      auto & output_robot = blue_robots_estimates.at(blue_id).value();
      Eigen::Vector2d output_angle{0, 0};
      double total_score = 0.0;

      for (const auto & robot_with_score : blue_robots_with_scores.at(blue_id)) {
        const Robot & robot = std::get<0>(robot_with_score);
        const double & score = std::get<1>(robot_with_score) + 1;

        output_robot.position += score * robot.position;
        output_angle += score * Eigen::Vector2d{std::cos(robot.theta), std::sin(
            robot.theta)};
        output_robot.velocity += score * robot.velocity;
        output_robot.omega += score * robot.omega;
        output_robot.acceleration += score * robot.acceleration;
        output_robot.alpha += score * robot.alpha;

        total_score += score;
      }

      output_robot.position /= total_score;
      output_robot.theta = 0.0;
      if (output_angle.norm() > 0) {
        output_robot.theta = std::atan2(output_angle.y(), output_angle.x());
      }
      output_robot.velocity /= total_score;
      output_robot.omega /= total_score;
      output_robot.acceleration /= total_score;
      output_robot.alpha /= total_score;
    }
  }

  return blue_robots_estimates;
}

ateam_msgs::msg::VisionStateCameraArray World::get_vision_world_state() const
{
  ateam_msgs::msg::VisionStateCameraArray world_state;
  for (const auto & camera_pair : cameras) {
    ateam_msgs::msg::VisionStateCamera camera_state = camera_pair.second.get_vision_camera_state();
    camera_state.camera_id = camera_pair.first;
    world_state.camera_states.push_back(camera_state);
  }

  return world_state;
}

}  // namespace ateam_vision_filter
