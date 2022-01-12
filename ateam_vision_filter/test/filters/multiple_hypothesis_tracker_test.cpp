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

#include "filters/multiple_hypothesis_tracker.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

TEST(MultipleHypothesisTracker, getStateEstimate_ShouldReturnNothing_WhenNoTracks)
{
  MultipleHypothesisTracker mht;

  std::optional<std::pair<Eigen::VectorXd, double>> ret = mht.get_state_estimate();

  EXPECT_FALSE(ret.has_value());
}

TEST(MultipleHypothesisTracker, getStateEstimate_ShouldReturnEstimate_WhenOneMeasuremnt)
{
  MultipleHypothesisTracker mht;
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{1.0, 1.0});
  kf.set_initial_p(Eigen::Matrix2d::Ones());
  kf.set_F(Models::Ball::F);
  kf.set_B(Models::Ball::B);
  kf.set_H(Models::Ball::H);
  kf.set_Q(Models::Ball::Q);
  kf.set_R(Models::Ball::R);
  std::vector<Models::ModelType> model_types{Models::ModelType::BALL_ROLLING_FRICTION};
  std::shared_ptr<ModelInputGenerator> model_input_generator =
    std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator =
    std::make_shared<TransmissionProbabilityGenerator>();

  InteractingMultipleModelFilter immf;
  immf.setup(kf, model_types, model_input_generator, transmission_probability_generator);
  mht.set_base_track(immf);

  std::vector<Eigen::VectorXd> measurements;
  measurements.push_back(Eigen::Vector2d{1, 2});
  mht.update(measurements);

  std::optional<std::pair<Eigen::VectorXd, double>> ret = mht.get_state_estimate();

  ASSERT_TRUE(ret.has_value());
  EXPECT_NEAR(ret.value().first.x(), 1.0, 1e-6);
  EXPECT_NEAR(ret.value().first.y(), 2.0, 1e-6);
}

TEST(
  MultipleHypothesisTracker,
  getStateEstimate_ShouldReturnEstimate_WhenOneMeasuremntThenOneMeasurement)
{
  MultipleHypothesisTracker mht;
  KalmanFilter kf;
  kf.set_initial_x_hat((Eigen::MatrixXd(6, 1) << 0, 0, 0, 0, 0, 0).finished());
  kf.set_initial_p(Eigen::MatrixXd::Ones(6, 6));
  kf.set_F(Models::Ball::F);
  kf.set_B(Models::Ball::B);
  kf.set_H(Models::Ball::H);
  kf.set_Q(Models::Ball::Q);
  kf.set_R(Models::Ball::R);
  std::vector<Models::ModelType> model_types{Models::ModelType::BALL_ROLLING_FRICTION};
  std::shared_ptr<ModelInputGenerator> model_input_generator =
    std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator =
    std::make_shared<TransmissionProbabilityGenerator>();

  InteractingMultipleModelFilter immf;
  immf.setup(kf, model_types, model_input_generator, transmission_probability_generator);
  mht.set_base_track(immf);

  std::vector<Eigen::VectorXd> measurements;
  measurements.push_back(Eigen::Vector2d{1, 2});
  mht.update(measurements);
  mht.update(measurements);

  std::optional<std::pair<Eigen::VectorXd, double>> ret = mht.get_state_estimate();

  ASSERT_TRUE(ret.has_value());
  EXPECT_NEAR(ret.value().first.x(), 1.0, 1e-6);
  EXPECT_NEAR(ret.value().first.y(), 2.0, 1e-6);
}

TEST(
  MultipleHypothesisTracker,
  getStateEstimate_ShouldReturnEstimate_WhenTwoMeasuremntThenTwoMeasurement)
{
  MultipleHypothesisTracker mht;
  KalmanFilter kf;
  kf.set_initial_x_hat((Eigen::MatrixXd(6, 1) << 0, 0, 0, 0, 0, 0).finished());
  kf.set_initial_p(Eigen::MatrixXd::Ones(6, 6));
  kf.set_F(Models::Ball::F);
  kf.set_B(Models::Ball::B);
  kf.set_H(Models::Ball::H);
  kf.set_Q(Models::Ball::Q);
  kf.set_R(Models::Ball::R);
  std::vector<Models::ModelType> model_types{Models::ModelType::BALL_ROLLING_FRICTION};
  std::shared_ptr<ModelInputGenerator> model_input_generator =
    std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator =
    std::make_shared<TransmissionProbabilityGenerator>();

  InteractingMultipleModelFilter immf;
  immf.setup(kf, model_types, model_input_generator, transmission_probability_generator);
  mht.set_base_track(immf);

  std::vector<Eigen::VectorXd> measurements;
  measurements.push_back(Eigen::Vector2d{1, 2});
  measurements.push_back(Eigen::Vector2d{4, 4});
  mht.update(measurements);
  mht.update(measurements);

  std::optional<std::pair<Eigen::VectorXd, double>> ret = mht.get_state_estimate();

  ASSERT_TRUE(ret.has_value());
  EXPECT_NEAR(ret.value().first.x(), 1.0, 1e-6);
  EXPECT_NEAR(ret.value().first.y(), 2.0, 1e-6);
}

TEST(
  MultipleHypothesisTracker,
  getStateEstimate_ShouldReturnBest_WhenOnlyOneGetsConstantUpdates)
{
  MultipleHypothesisTracker mht;
  KalmanFilter kf;
  kf.set_initial_x_hat((Eigen::MatrixXd(6, 1) << 0, 0, 0, 0, 0, 0).finished());
  kf.set_initial_p(Eigen::MatrixXd::Ones(6, 6));
  kf.set_F(Models::Ball::F);
  kf.set_B(Models::Ball::B);
  kf.set_H(Models::Ball::H);
  kf.set_Q(Models::Ball::Q);
  kf.set_R(Models::Ball::R);
  std::vector<Models::ModelType> model_types{Models::ModelType::BALL_ROLLING_FRICTION};
  std::shared_ptr<ModelInputGenerator> model_input_generator =
    std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator =
    std::make_shared<TransmissionProbabilityGenerator>();

  InteractingMultipleModelFilter immf;
  immf.setup(kf, model_types, model_input_generator, transmission_probability_generator);
  mht.set_base_track(immf);

  std::vector<Eigen::VectorXd> measurements1;
  measurements1.push_back(Eigen::Vector2d{1, 2});
  measurements1.push_back(Eigen::Vector2d{4, 4});
  std::vector<Eigen::VectorXd> measurements2;
  measurements2.push_back(Eigen::Vector2d{4, 4});

  for (int i = 0; i < 30; i++) {
    mht.update(measurements1);
    mht.predict();
    mht.update(measurements2);
    mht.predict();
  }
  std::optional<std::pair<Eigen::VectorXd, double>> ret = mht.get_state_estimate();

  ASSERT_TRUE(ret.has_value());
  EXPECT_NEAR(ret.value().first.x(), 4.0, 1e-6);
  EXPECT_NEAR(ret.value().first.y(), 4.0, 1e-6);
}

TEST(
  MultipleHypothesisTracker,
  getStateEstimate_ShouldReturnNone_WhenOneStoppedGettingUpdates)
{
  MultipleHypothesisTracker mht;
  KalmanFilter kf;
  kf.set_initial_x_hat((Eigen::MatrixXd(6, 1) << 0, 0, 0, 0, 0, 0).finished());
  kf.set_initial_p(Eigen::MatrixXd::Ones(6, 6));
  kf.set_F(Models::Ball::F);
  kf.set_B(Models::Ball::B);
  kf.set_H(Models::Ball::H);
  kf.set_Q(Models::Ball::Q);
  kf.set_R(Models::Ball::R);
  std::vector<Models::ModelType> model_types{Models::ModelType::BALL_ROLLING_FRICTION};
  std::shared_ptr<ModelInputGenerator> model_input_generator =
    std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator =
    std::make_shared<TransmissionProbabilityGenerator>();

  InteractingMultipleModelFilter immf;
  immf.setup(kf, model_types, model_input_generator, transmission_probability_generator);
  mht.set_base_track(immf);

  std::vector<Eigen::VectorXd> measurements1;
  measurements1.push_back(Eigen::Vector2d{1, 2});

  for (int i = 0; i < 20; i++) {
    mht.update(measurements1);
    mht.predict();
  }
  for (int i = 0; i < 200; i++) {
    mht.predict();
  }
  std::optional<std::pair<Eigen::VectorXd, double>> ret = mht.get_state_estimate();

  ASSERT_FALSE(ret.has_value());
}
