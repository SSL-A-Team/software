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

#include <ateam_vision_filter/camera.hpp>

TEST(Camera, getEstimateWithScore_ShouldReturnNullopt_WhenNoData)
{
  std::shared_ptr<ModelInputGenerator> mig = std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> tpg =
    std::make_shared<TransmissionProbabilityGenerator>();
  Camera camera(mig, tpg);

  auto ball_with_score = camera.get_ball_estimate_with_score();
  auto yellow_robots_with_score = camera.get_yellow_robot_estimates_with_score();
  auto blue_robots_with_score = camera.get_blue_robot_estimates_with_score();

  EXPECT_FALSE(ball_with_score.has_value());
  for (const auto & robot : yellow_robots_with_score) {
    EXPECT_FALSE(robot.has_value());
  }
  for (const auto & robot : blue_robots_with_score) {
    EXPECT_FALSE(robot.has_value());
  }
}

TEST(Camera, getBallEstimateWithScore_ShouldReturnData_WhenPredictAndUpdate)
{
  std::shared_ptr<ModelInputGenerator> mig = std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> tpg =
    std::make_shared<TransmissionProbabilityGenerator>();
  Camera camera(mig, tpg);
  BallMeasurement ball_measurement;
  std::vector<BallMeasurement> ball_measurements;
  CameraMeasurement camera_measurement;

  ball_measurement.position = Eigen::Vector2d{1, 3};
  ball_measurements.push_back(ball_measurement);
  camera_measurement.ball = ball_measurements;

  camera.update(camera_measurement);
  camera.predict();

  auto ball_with_score = camera.get_ball_estimate_with_score();

  ASSERT_TRUE(ball_with_score.has_value());
  EXPECT_NEAR(std::get<0>(ball_with_score.value()).position.x(), 1, 1e-6);
  EXPECT_NEAR(std::get<0>(ball_with_score.value()).position.y(), 3, 1e-6);
  EXPECT_NEAR(std::get<0>(ball_with_score.value()).velocity.norm(), 0, 1e-6);
  EXPECT_NEAR(std::get<0>(ball_with_score.value()).acceleration.norm(), 0, 1e-6);
}

TEST(Camera, getBallEstimateWithScore_ShouldReturnBestData_WhenConsistentAndInconsistentData)
{
  std::shared_ptr<ModelInputGenerator> mig = std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> tpg =
    std::make_shared<TransmissionProbabilityGenerator>();
  Camera camera(mig, tpg);
  BallMeasurement ball_measurement1, ball_measurement2;
  std::vector<BallMeasurement> ball_measurements1, ball_measurements2;
  CameraMeasurement camera_measurement1, camera_measurement2;

  ball_measurement1.position = Eigen::Vector2d{1, 3};
  ball_measurement2.position = Eigen::Vector2d{4, 8};
  ball_measurements1.push_back(ball_measurement1);
  ball_measurements1.push_back(ball_measurement2);
  ball_measurements2.push_back(ball_measurement1);
  camera_measurement1.ball = ball_measurements1;
  camera_measurement2.ball = ball_measurements2;

  for (size_t i = 0; i < 100; i++) {
    camera.update(camera_measurement1);
    camera.predict();
    camera.update(camera_measurement2);
    camera.predict();
  }

  auto ball_with_score = camera.get_ball_estimate_with_score();

  ASSERT_TRUE(ball_with_score.has_value());
  EXPECT_NEAR(std::get<0>(ball_with_score.value()).position.x(), 1, 1e-6);
  EXPECT_NEAR(std::get<0>(ball_with_score.value()).position.y(), 3, 1e-6);
  EXPECT_NEAR(std::get<0>(ball_with_score.value()).velocity.norm(), 0, 1e-6);
  EXPECT_NEAR(std::get<0>(ball_with_score.value()).acceleration.norm(), 0, 1e-6);
}

TEST(Camera, getRobotEstimatesWithScore_ShouldReturnData_WhenPredictAndUpdate)
{
  std::shared_ptr<ModelInputGenerator> mig = std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> tpg =
    std::make_shared<TransmissionProbabilityGenerator>();
  Camera camera(mig, tpg);
  RobotMeasurement robot_measurement;
  std::array<std::vector<RobotMeasurement>, 16> robot_measurements;
  CameraMeasurement camera_measurement;

  robot_measurement.position = Eigen::Vector2d{1, 3};
  robot_measurement.theta = 5;
  robot_measurements.at(1).push_back(robot_measurement);
  camera_measurement.blue_robots = robot_measurements;
  camera_measurement.yellow_robots = robot_measurements;

  camera.update(camera_measurement);
  camera.predict();

  auto robots_with_score = camera.get_yellow_robot_estimates_with_score();

  for (size_t i = 0; i < 16; i++) {
    if (i == 1) {
      ASSERT_TRUE(robots_with_score.at(i).has_value());
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).position.x(), 1, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).position.y(), 3, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).theta, 5, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).velocity.norm(), 0, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).omega, 0, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).acceleration.norm(), 0, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).alpha, 0, 1e-6);
    } else {
      EXPECT_FALSE(robots_with_score.at(i).has_value());
    }
  }
}

TEST(Camera, getRobotEstimatesWithScore_ShouldReturnBestData_WhenConsistentAndInconsistentData)
{
  std::shared_ptr<ModelInputGenerator> mig = std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> tpg =
    std::make_shared<TransmissionProbabilityGenerator>();
  Camera camera(mig, tpg);
  RobotMeasurement robot_measurement1, robot_measurement2;
  std::array<std::vector<RobotMeasurement>, 16> robot_measurements1, robot_measurements2;
  CameraMeasurement camera_measurement1, camera_measurement2;

  robot_measurement1.position = Eigen::Vector2d{1, 3};
  robot_measurement1.theta = 5;
  robot_measurement2.position = Eigen::Vector2d{7, 9};
  robot_measurement2.theta = 11;
  robot_measurements1.at(1).push_back(robot_measurement1);
  robot_measurements1.at(1).push_back(robot_measurement2);
  robot_measurements2.at(1).push_back(robot_measurement1);
  camera_measurement1.blue_robots = robot_measurements1;
  camera_measurement1.yellow_robots = robot_measurements1;
  camera_measurement2.blue_robots = robot_measurements2;
  camera_measurement2.yellow_robots = robot_measurements2;

  for (size_t i = 0; i < 100; i++) {
    camera.update(camera_measurement1);
    camera.predict();
    camera.update(camera_measurement2);
    camera.predict();
  }

  auto robots_with_score = camera.get_yellow_robot_estimates_with_score();

  for (size_t i = 0; i < 16; i++) {
    if (i == 1) {
      ASSERT_TRUE(robots_with_score.at(i).has_value());
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).position.x(), 1, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).position.y(), 3, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).theta, 5, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).velocity.norm(), 0, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).omega, 0, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).acceleration.norm(), 0, 1e-6);
      EXPECT_NEAR(std::get<0>(robots_with_score.at(i).value()).alpha, 0, 1e-6);
    } else {
      EXPECT_FALSE(robots_with_score.at(i).has_value());
    }
  }
}
