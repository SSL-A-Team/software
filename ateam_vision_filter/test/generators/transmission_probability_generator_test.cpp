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

// #include <gtest/gtest.h>

// #include <optional>
// #include <array>

// #include "generators/transmission_probability_generator.hpp"
// #include "types/ball.hpp"
// #include "types/models.hpp"
// #include "types/robot.hpp"

// using namespace ateam_vision_filter;  // NOLINT(build/namespaces)

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn085_WhenSameModelToFrom)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 11> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_STOP_ON_DRIBBLER,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   for (const auto & model : models) {
//     EXPECT_NEAR(tpg.get_transmission_probability(Eigen::Vector4d(), model, model), 0.85, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn015_WhenSlidingToRolling)
// {
//   TransmissionProbabilityGenerator tpg;

//   EXPECT_NEAR(
//     tpg.get_transmission_probability(
//       Eigen::Vector4d(),
//       Models::ModelType::BALL_SLIDING_FRICTION,
//       Models::ModelType::BALL_ROLLING_FRICTION), 0.15, 1e-6);
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenNoRobotForBounce)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 10> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_STOP_ON_DRIBBLER,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 0, 0, 0, 0, 0, 0;

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_BOUNCE_ON_ROBOT), 0.0, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn015_WhenCloseAndMovingToRobotForBounce)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 10> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_STOP_ON_DRIBBLER,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 0.1, 0, -1, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_BOUNCE_ON_ROBOT), 0.15, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenCloseAndMovingAwayRobotForBounce)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 10> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_STOP_ON_DRIBBLER,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 0.1, 0, 1, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_BOUNCE_ON_ROBOT), 0.0, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenFarAndMovingToRobotForBounce)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 10> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_STOP_ON_DRIBBLER,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 1, 0, -1, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_BOUNCE_ON_ROBOT), 0.0, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenNoRobotForStop)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 10> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 1, 0, -1, 0, 0, 0;

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_STOP_ON_DRIBBLER), 0.0, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn015_WhenCloseInMouthMovingToRobotForStop)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 10> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 0.1, 0, -1, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};
//   blue_robots.at(0).value().theta = 0;

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_STOP_ON_DRIBBLER), 0.15, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenFarInMouthMovingToRobotForStop)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 10> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 1, 0, -1, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};
//   blue_robots.at(0).value().theta = 0;

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_STOP_ON_DRIBBLER), 0.0, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenCloseOutMouthUpMovingToRobotForStop)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 10> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 0, 0.1, 0, -1, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};
//   blue_robots.at(0).value().theta = 0;

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_STOP_ON_DRIBBLER), 0.0, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenCloseOutMouthLeftMovingToRobotForStop)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 10> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << -0.1, 0, 1, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};
//   blue_robots.at(0).value().theta = 0;

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_STOP_ON_DRIBBLER), 0.0, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenCloseInMouthMovingAwayRobotForStop)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 10> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 0.1, 0, 1, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};
//   blue_robots.at(0).value().theta = 0;

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_STOP_ON_DRIBBLER), 0.0, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenNoRobotForKick)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 8> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_STOP_ON_DRIBBLER,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};
//   std::array<Models::ModelType, 3> kicks{
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 0.1, 0, 1, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     for (const auto & kick : kicks) {
//       EXPECT_NEAR(
//         tpg.get_transmission_probability(
//           ball_state,
//           model,
//           kick), 0.0, 1e-6);
//     }
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn015_WhenCloseInMouthRobotForKick)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 8> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_STOP_ON_DRIBBLER,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};
//   std::array<Models::ModelType, 3> kicks{
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 0.1, 0, 0, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};
//   blue_robots.at(0).value().theta = 0;

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     for (const auto & kick : kicks) {
//       EXPECT_NEAR(
//         tpg.get_transmission_probability(
//           ball_state,
//           model,
//           kick), 0.15, 1e-6);
//     }
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenFarInMouthRobotForKick)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 8> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_STOP_ON_DRIBBLER,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};
//   std::array<Models::ModelType, 3> kicks{
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK};

//   Eigen::VectorXd ball_state(6);
//   ball_state << 1, 0, 0, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};
//   blue_robots.at(0).value().theta = 0;

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     for (const auto & kick : kicks) {
//       EXPECT_NEAR(
//         tpg.get_transmission_probability(
//           ball_state,
//           model,
//           kick), 0.0, 1e-6);
//     }
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenCloseOutMouthRobotForKick)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 8> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_SLIDING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_STOP_ON_DRIBBLER,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};
//   std::array<Models::ModelType, 3> kicks{
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK};

//   Eigen::VectorXd ball_state(6);
//   ball_state << -0.1, 0, 0, 0, 0, 0;

//   std::array<std::optional<Robot>, 16> blue_robots;
//   std::array<std::optional<Robot>, 16> yellow_robots;
//   std::optional<Ball> ball;

//   blue_robots.at(0) = Robot();
//   blue_robots.at(0).value().position = Eigen::Vector2d{0, 0};
//   blue_robots.at(0).value().theta = 0;

//   tpg.update(blue_robots, yellow_robots, ball);

//   for (const auto & model : models) {
//     for (const auto & kick : kicks) {
//       EXPECT_NEAR(
//         tpg.get_transmission_probability(
//           ball_state,
//           model,
//           kick), 0.0, 1e-6);
//     }
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   WShouldReturn085_WhenKickForSliding)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 3> kicks{
//     Models::ModelType::BALL_SLOW_KICK,
//     Models::ModelType::BALL_MEDIUM_KICK,
//     Models::ModelType::BALL_FAST_KICK};

//   Eigen::VectorXd ball_state(6);
//   ball_state << -0.1, 0, 0, 0, 0, 0;

//   for (const auto & kick : kicks) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         kick,
//         Models::ModelType::BALL_SLIDING_FRICTION), 0.85, 1e-6);
//   }
// }

// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn0_WhenNotKickForSliding)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 7> models{
//     Models::ModelType::TEST_EMPTY_MODEL,
//     Models::ModelType::BALL_ROLLING_FRICTION,
//     Models::ModelType::BALL_BOUNCE_ON_ROBOT,
//     Models::ModelType::BALL_STOP_ON_DRIBBLER,
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd ball_state(6);
//   ball_state << -0.1, 0, 0, 0, 0, 0;

//   for (const auto & model : models) {
//     EXPECT_NEAR(
//       tpg.get_transmission_probability(
//         ball_state,
//         model,
//         Models::ModelType::BALL_SLIDING_FRICTION), 0.0, 1e-6);
//   }
// }


// TEST(
//   TransmissionProbabilityGenerator,
//   getTransmissionProbability_ShouldReturn015Or085_WhenNotSameRobotModelOrSame)
// {
//   TransmissionProbabilityGenerator tpg;
//   std::array<Models::ModelType, 3> models{
//     Models::ModelType::ROBOT_NO_ACCEL,
//     Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL,
//     Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL};

//   Eigen::VectorXd robot_state(9);
//   robot_state << 0, 0, 0, 0, 0, 0, 0, 0, 0;

//   for (const auto & model1 : models) {
//     for (const auto & model2 : models) {
//       double expected = 0.85;
//       if (model1 != model2) {
//         expected = 0.15;
//       }

//       EXPECT_NEAR(
//         tpg.get_transmission_probability(
//           robot_state,
//           model1,
//           model2), expected, 1e-6);
//     }
//   }
// }
