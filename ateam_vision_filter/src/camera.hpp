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

#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include <optional>

#include <array>
#include <memory>
#include <utility>
#include <vector>

#include <ateam_msgs/msg/vision_camera_state.hpp>

#include "filters/multiple_hypothesis_tracker.hpp"
#include "generators/model_input_generator.hpp"
#include "generators/transmission_probability_generator.hpp"
#include "types/camera_measurement.hpp"

class Camera
{
public:
  using BallWithScore = std::pair<Ball, double>;
  using RobotWithScore = std::pair<Robot, double>;

  Camera(
    std::shared_ptr<ModelInputGenerator> model_input_generator,
    std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator);

  /**
   * Updates the camera with a specific frame's measurement
   *
   * @param camera_measurement Measurement from a specific frame
   */
  void update(const CameraMeasurement & camera_measurement);

  /**
   * Step froward the camera physics models one time step
   */
  void predict();

  /**
   * @return Returns a ball with a corresponding likelihood score (if exists)
   */
  std::optional<BallWithScore> get_ball_estimate_with_score();

  /**
   * @return Returns yellow robots with a corresponding likelihood score (if exists)
   */
  std::array<std::optional<RobotWithScore>, 16> get_yellow_robot_estimates_with_score();

  /**
   * @return Returns blue robots with a corresponding likelihood score (if exists)
   */
  std::array<std::optional<RobotWithScore>, 16> get_blue_robot_estimates_with_score();

  /**
   * @return ROS2 msg containing the current internal state
   */
  ateam_msgs::msg::VisionCameraState get_vision_camera_state() const;

private:
  void setup_ball_interacting_multiple_model_filter(
    std::shared_ptr<ModelInputGenerator> model_input_generator,
    std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator);
  void setup_robot_interacting_multiple_model_filter(
    std::shared_ptr<ModelInputGenerator> model_input_generator,
    std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator);

  static std::vector<Eigen::VectorXd> robot_measurements_to_vector(
    const std::vector<RobotMeasurement> & robot_measurements);

  static std::vector<Eigen::VectorXd> ball_measurements_to_vector(
    const std::vector<BallMeasurement> & ball_measurements);

  static std::array<std::optional<RobotWithScore>, 16> get_robot_estimates_with_score(
    const std::array<MultipleHypothesisTracker, 16> robot_team);

  std::array<MultipleHypothesisTracker, 16> yellow_team;
  std::array<MultipleHypothesisTracker, 16> blue_team;
  MultipleHypothesisTracker ball;
};

#endif  // CAMERA_HPP_
