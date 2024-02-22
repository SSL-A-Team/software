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

#ifndef WORLD_HPP_
#define WORLD_HPP_

#include <optional>

#include <array>
#include <map>
#include <memory>

#include <ateam_msgs/msg/vision_world_state.hpp>

#include "camera.hpp"
#include "generators/model_input_generator.hpp"
#include "generators/transmission_probability_generator.hpp"
#include "types/ball.hpp"
#include "types/camera_measurement.hpp"
#include "types/robot.hpp"

namespace ateam_vision_filter
{

class World
{
public:
  using CameraID = int;

  World();

  /**
   * Updates the world with a specific camera's measurement
   *
   * @param cameraID Unique ID of the camera given by SSL Vision
   * @param measurement Measurement from the camera frame
   */
  void update_camera(const CameraID & cameraID, const CameraMeasurement & measurement);

  /**
   * Step forward the world physics models one time step
   */
  void predict();

  /**
   * @return The best possible estimate for the ball (if one exists)
   */
  std::optional<Ball> get_ball_estimate();

  /**
   * @return The best possible estimate for each yellow robot (if one exists)
   */
  std::array<std::optional<Robot>, 16> get_yellow_robots_estimate();

  /**
   * @return The best possible estimate for each blue robot (if one exists)
   */
  std::array<std::optional<Robot>, 16> get_blue_robots_estimate();

  /**
   * @return ROS2 msg containing the current internal state
   */
  ateam_msgs::msg::VisionWorldState get_vision_world_state() const;

private:
  std::shared_ptr<ModelInputGenerator> model_input_generator;
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator;
  std::map<CameraID, Camera> cameras;
};

}  // namespace ateam_vision_filter

#endif  // WORLD_HPP_
