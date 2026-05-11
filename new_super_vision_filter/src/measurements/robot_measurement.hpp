// Copyright 2025 A Team
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

#ifndef ROBOT_MEASUREMENT_HPP_
#define ROBOT_MEASUREMENT_HPP_

#include <ateam_common/game_controller_listener.hpp>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include <angles/angles.h>

#include "filter_types.hpp"

/**
 * @brief Robot position and angle measurement from a single camera detection,
 * used to provide updates to the Kalman filter.
 */
class RobotMeasurement {
public:
  RobotMeasurement(
    const ssl_league_msgs::msg::VisionDetectionRobot & bot_detection,
    int & camera_id,
    ateam_common::TeamColor & team
  )
  : camera_id(camera_id), team(team)
  {
    pos << bot_detection.pose.position.x,
      bot_detection.pose.position.y;
    angle << bot_detection.pose.orientation.w;
    robot_id = bot_detection.robot_id;
    timestamp = std::chrono::steady_clock::now();
  }

  int getId() const
  {
    return robot_id;
  }

  void invert()
  {
    pos *= -1.0;
    // A bit kludgy... since we use templating of Eigen types for measurements,
    // this seemed like the easiest way to modify the data and ensure it's
    // clamped properly without writing a bunch of extra functions/code
    double new_angle = angles::normalize_angle_positive(angle.w() + M_PI);
    AngleMeasurement new_angle_measurement;
    new_angle_measurement << new_angle;
    angle = new_angle_measurement;
  }

  std::chrono::time_point<std::chrono::steady_clock> getTimestamp() const
  {
    return timestamp;
  }

  PosMeasurement pos;
  AngleMeasurement angle;

private:
  std::chrono::time_point<std::chrono::steady_clock> timestamp;
  int camera_id;
  ateam_common::TeamColor team;
  int robot_id;

};

#endif // ROBOT_MEASUREMENT_HPP_
