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

#include <array>
#include <optional>

#include "../src/generators/model_input_generator.hpp"
#include "../src/types/ball.hpp"
#include "../src/types/models.hpp"
#include "../src/types/robot.hpp"

TEST(model_input_generator, get_model_input_ShouldReturnSmallerVelocity_WhenBallRollingFriction)
{
  ModelInputGenerator mig;

  Eigen::VectorXd input_state(6);
  input_state(0) = 0.0;
  input_state(1) = 0.0;
  input_state(2) = 1.0;
  input_state(3) = 0.0;
  input_state(4) = 0.0;
  input_state(5) = 0.0;

  Eigen::VectorXd ret = mig.get_model_input(input_state, Models::ModelType::BALL_ROLLING_FRICTION);
  EXPECT_NEAR(ret(0), input_state(0), 1e-6);
  EXPECT_NEAR(ret(1), input_state(1), 1e-6);
  EXPECT_LT(ret(2), input_state(2));
  EXPECT_GE(ret(2), 0.0);
  EXPECT_NEAR(ret(3), input_state(3), 1e-6);
  EXPECT_NEAR(ret(4), input_state(4), 1e-6);
  EXPECT_NEAR(ret(5), input_state(5), 1e-6);
}

TEST(model_input_generator, get_model_input_ShouldReturnSmallerVelocity_WhenBallSlidingFriction)
{
  ModelInputGenerator mig;

  Eigen::VectorXd input_state(6);
  input_state(0) = 0.0;
  input_state(1) = 0.0;
  input_state(2) = 1.0;
  input_state(3) = 0.0;
  input_state(4) = 0.0;
  input_state(5) = 0.0;

  Eigen::VectorXd ret = mig.get_model_input(input_state, Models::ModelType::BALL_SLIDING_FRICTION);
  EXPECT_NEAR(ret(0), input_state(0), 1e-6);
  EXPECT_NEAR(ret(1), input_state(1), 1e-6);
  EXPECT_LT(ret(2), input_state(2));
  EXPECT_GE(ret(2), 0.0);
  EXPECT_NEAR(ret(3), input_state(3), 1e-6);
  EXPECT_NEAR(ret(4), input_state(4), 1e-6);
  EXPECT_NEAR(ret(5), input_state(5), 1e-6);
}

TEST(model_input_generator, get_model_input_ShouldReturn0_WhenBallBounceWithNoRobots)
{
  ModelInputGenerator mig;
  std::array<std::optional<Robot>, 16> blue_robots;
  std::array<std::optional<Robot>, 16> yellow_robots;
  std::optional<Ball> ball;
  mig.update(blue_robots, yellow_robots, ball);

  Eigen::VectorXd input_state(6);
  input_state(0) = 1.0;
  input_state(1) = 2.0;
  input_state(2) = 3.0;
  input_state(3) = 4.0;
  input_state(4) = 5.0;
  input_state(5) = 6.0;

  Eigen::VectorXd ret = mig.get_model_input(input_state, Models::ModelType::BALL_BOUNCE_ON_ROBOT);
  EXPECT_NEAR(ret(0), 0.0, 1e-6);
  EXPECT_NEAR(ret(1), 0.0, 1e-6);
  EXPECT_NEAR(ret(2), 0.0, 1e-6);
  EXPECT_NEAR(ret(3), 0.0, 1e-6);
  EXPECT_NEAR(ret(4), 0.0, 1e-6);
  EXPECT_NEAR(ret(5), 0.0, 1e-6);
}

TEST(model_input_generator, get_model_input_ShouldReturnNegVelocity_WhenBallSettle)
{
  ModelInputGenerator mig;

  Eigen::VectorXd input_state(6);
  input_state(0) = 1.0;
  input_state(1) = 2.0;
  input_state(2) = 3.0;
  input_state(3) = 4.0;
  input_state(4) = 5.0;
  input_state(5) = 6.0;

  Eigen::VectorXd ret = mig.get_model_input(input_state, Models::ModelType::BALL_STOP_ON_DRIBBLER);
  EXPECT_NEAR(ret(0), -input_state(2) - Models::dt * input_state(4), 1e-6);
  EXPECT_NEAR(ret(1), -input_state(3) - Models::dt * input_state(5), 1e-6);
  EXPECT_NEAR(ret(2), 0.0, 1e-6);
  EXPECT_NEAR(ret(3), 0.0, 1e-6);
  EXPECT_NEAR(ret(4), 0.0, 1e-6);
  EXPECT_NEAR(ret(5), 0.0, 1e-6);
}

TEST(model_input_generator, get_model_input_ShouldReturnZero_WhenRobotNoAccel)
{
  ModelInputGenerator mig;

  Eigen::VectorXd input_state(9);
  input_state(0) = 1.0;
  input_state(1) = 2.0;
  input_state(2) = 3.0;
  input_state(3) = 4.0;
  input_state(4) = 5.0;
  input_state(5) = 6.0;
  input_state(6) = 7.0;
  input_state(7) = 8.0;
  input_state(8) = 9.0;

  Eigen::VectorXd ret = mig.get_model_input(input_state, Models::ModelType::ROBOT_NO_ACCEL);
  EXPECT_NEAR(ret(0), 0.0, 1e-6);
  EXPECT_NEAR(ret(1), 0.0, 1e-6);
  EXPECT_NEAR(ret(2), 0.0, 1e-6);
  EXPECT_NEAR(ret(3), 0.0, 1e-6);
  EXPECT_NEAR(ret(4), 0.0, 1e-6);
  EXPECT_NEAR(ret(5), 0.0, 1e-6);
  EXPECT_NEAR(ret(6), 0.0, 1e-6);
  EXPECT_NEAR(ret(7), 0.0, 1e-6);
  EXPECT_NEAR(ret(8), 0.0, 1e-6);
}
