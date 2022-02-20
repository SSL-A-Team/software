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

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "world.hpp"

TEST(World, getEstimate_ShouldReturnNullopt_WhenNoData)
{
  World world;

  auto ball_estimate = world.get_ball_estimate();
  auto yellow_robots_estimate = world.get_yellow_robots_estimate();
  auto blue_robots_estimate = world.get_blue_robots_estimate();

  EXPECT_FALSE(ball_estimate.has_value());
  for (const auto & robot : yellow_robots_estimate) {
    EXPECT_FALSE(robot.has_value());
  }
  for (const auto & robot : blue_robots_estimate) {
    EXPECT_FALSE(robot.has_value());
  }
}

TEST(World, getBallEstimate_ShouldReturnData_WhenOneCameraConsistent)
{
  World world;
  BallMeasurement ball_measurement;
  std::vector<BallMeasurement> ball_measurements;
  CameraMeasurement camera_measurement;

  ball_measurement.position = Eigen::Vector2d{1, 2};
  ball_measurements.push_back(ball_measurement);
  camera_measurement.ball = ball_measurements;

  world.update_camera(0, camera_measurement);
  world.predict();

  auto ball_estimate = world.get_ball_estimate();

  ASSERT_TRUE(ball_estimate.has_value());
  EXPECT_NEAR(ball_estimate.value().position.x(), 1, 1e-6);
  EXPECT_NEAR(ball_estimate.value().position.y(), 2, 1e-6);
  EXPECT_NEAR(ball_estimate.value().velocity.norm(), 0, 1e-6);
  EXPECT_NEAR(ball_estimate.value().acceleration.norm(), 0, 1e-6);
}

TEST(World, getBallEstimate_ShouldReturnAverageData_WhenTwoCameraConsistent)
{
  World world;
  BallMeasurement ball_measurement1, ball_measurement2;
  std::vector<BallMeasurement> ball_measurements1, ball_measurements2;
  CameraMeasurement camera_measurement1, camera_measurement2;

  ball_measurement1.position = Eigen::Vector2d{1, 2};
  ball_measurements1.push_back(ball_measurement1);
  camera_measurement1.ball = ball_measurements1;
  ball_measurement2.position = Eigen::Vector2d{2, 4};
  ball_measurements2.push_back(ball_measurement2);
  camera_measurement2.ball = ball_measurements2;

  world.update_camera(0, camera_measurement1);
  world.update_camera(1, camera_measurement2);
  world.predict();

  auto ball_estimate = world.get_ball_estimate();

  ASSERT_TRUE(ball_estimate.has_value());
  EXPECT_NEAR(ball_estimate.value().position.x(), 1.5, 1e-6);
  EXPECT_NEAR(ball_estimate.value().position.y(), 3, 1e-6);
  EXPECT_NEAR(ball_estimate.value().velocity.norm(), 0, 1e-6);
  EXPECT_NEAR(ball_estimate.value().acceleration.norm(), 0, 1e-6);
}

TEST(World, getRobotEstimate_ShouldReturnData_WhenOneCameraConsistent)
{
  World world;
  RobotMeasurement robot_measurement;
  std::array<std::vector<RobotMeasurement>, 16> robot_measurements;
  CameraMeasurement camera_measurement;

  robot_measurement.position = Eigen::Vector2d{1, 2};
  robot_measurement.theta = 1.5;
  robot_measurements.at(1).push_back(robot_measurement);
  camera_measurement.yellow_robots = robot_measurements;
  camera_measurement.blue_robots = robot_measurements;

  world.update_camera(0, camera_measurement);
  world.predict();

  auto yellow_robots_estimate = world.get_yellow_robots_estimate();
  auto blue_robots_estimate = world.get_blue_robots_estimate();

  for (size_t i = 0; i < 16; i++) {
    auto robot = yellow_robots_estimate.at(i);
    if (i == 1) {
      ASSERT_TRUE(robot.has_value());
      EXPECT_NEAR(robot.value().position.x(), 1, 1e-6);
      EXPECT_NEAR(robot.value().position.y(), 2, 1e-6);
      EXPECT_NEAR(robot.value().theta, 1.5, 1e-6);
      EXPECT_NEAR(robot.value().velocity.norm(), 0, 1e-6);
      EXPECT_NEAR(robot.value().omega, 0, 1e-6);
      EXPECT_NEAR(robot.value().acceleration.norm(), 0, 1e-6);
      EXPECT_NEAR(robot.value().alpha, 0, 1e-6);
    } else {
      EXPECT_FALSE(robot.has_value());
    }
  }

  for (size_t i = 0; i < 16; i++) {
    auto robot = blue_robots_estimate.at(i);
    if (i == 1) {
      ASSERT_TRUE(robot.has_value());
      EXPECT_NEAR(robot.value().position.x(), 1, 1e-6);
      EXPECT_NEAR(robot.value().position.y(), 2, 1e-6);
      EXPECT_NEAR(robot.value().theta, 1.5, 1e-6);
      EXPECT_NEAR(robot.value().velocity.norm(), 0, 1e-6);
      EXPECT_NEAR(robot.value().omega, 0, 1e-6);
      EXPECT_NEAR(robot.value().acceleration.norm(), 0, 1e-6);
      EXPECT_NEAR(robot.value().alpha, 0, 1e-6);
    } else {
      EXPECT_FALSE(robot.has_value());
    }
  }
}

TEST(World, getRobotEstimate_ShouldReturnAverageData_WhenTwoCameraConsistent)
{
  World world;
  RobotMeasurement robot_measurement1, robot_measurement2;
  std::array<std::vector<RobotMeasurement>, 16> robot_measurements1, robot_measurements2;
  CameraMeasurement camera_measurement1, camera_measurement2;

  robot_measurement1.position = Eigen::Vector2d{1, 2};
  robot_measurement1.theta = 1.5;
  robot_measurements1.at(1).push_back(robot_measurement1);
  camera_measurement1.yellow_robots = robot_measurements1;
  camera_measurement1.blue_robots = robot_measurements1;

  robot_measurement2.position = Eigen::Vector2d{4, 5};
  robot_measurement2.theta = 0.5;
  robot_measurements2.at(1).push_back(robot_measurement2);
  camera_measurement2.yellow_robots = robot_measurements2;
  camera_measurement2.blue_robots = robot_measurements2;


  world.update_camera(0, camera_measurement1);
  world.update_camera(1, camera_measurement2);
  world.predict();

  auto yellow_robots_estimate = world.get_yellow_robots_estimate();
  auto blue_robots_estimate = world.get_blue_robots_estimate();

  for (size_t i = 0; i < 16; i++) {
    auto robot = yellow_robots_estimate.at(i);
    if (i == 1) {
      ASSERT_TRUE(robot.has_value());
      EXPECT_NEAR(robot.value().position.x(), 2.5, 1e-6);
      EXPECT_NEAR(robot.value().position.y(), 3.5, 1e-6);
      EXPECT_NEAR(robot.value().theta, 1.0, 1e-6);
      EXPECT_NEAR(robot.value().velocity.norm(), 0, 1e-6);
      EXPECT_NEAR(robot.value().omega, 0, 1e-6);
      EXPECT_NEAR(robot.value().acceleration.norm(), 0, 1e-6);
      EXPECT_NEAR(robot.value().alpha, 0, 1e-6);
    } else {
      EXPECT_FALSE(robot.has_value());
    }
  }

  for (size_t i = 0; i < 16; i++) {
    auto robot = blue_robots_estimate.at(i);
    if (i == 1) {
      ASSERT_TRUE(robot.has_value());
      EXPECT_NEAR(robot.value().position.x(), 2.5, 1e-6);
      EXPECT_NEAR(robot.value().position.y(), 3.5, 1e-6);
      EXPECT_NEAR(robot.value().theta, 1.0, 1e-6);
      EXPECT_NEAR(robot.value().velocity.norm(), 0, 1e-6);
      EXPECT_NEAR(robot.value().omega, 0, 1e-6);
      EXPECT_NEAR(robot.value().acceleration.norm(), 0, 1e-6);
      EXPECT_NEAR(robot.value().alpha, 0, 1e-6);
    } else {
      EXPECT_FALSE(robot.has_value());
    }
  }
}
