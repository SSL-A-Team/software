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

#include "filters/kalman_filter.hpp"
#include "filters/interacting_multiple_model_filter.hpp"
#include "generators/model_input_generator.hpp"
#include "generators/transmission_probability_generator.hpp"
#include "types/models.hpp"

TEST(InteractingMultipleModelFilter, getStateEstimate_ShouldReturnIntial_WhenNothingChanges)
{
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{1.0, 2.0});
  std::vector<Models::ModelType> model_types{Models::ModelType::TEST_EMPTY_MODEL};
  std::shared_ptr<ModelInputGenerator> model_input_generator =
    std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator =
    std::make_shared<TransmissionProbabilityGenerator>();

  InteractingMultipleModelFilter imm;
  imm.setup(kf, model_types, model_input_generator, transmission_probability_generator);

  EXPECT_FLOAT_EQ(imm.get_state_estimate().x(), 1.0);
  EXPECT_FLOAT_EQ(imm.get_state_estimate().y(), 2.0);
}

TEST(InteractingMultipleModelFilter, predict_ShouldReturnX10Step_WhenPredictX10)
{
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{1.0, 1.0});
  kf.set_initial_p(Eigen::Matrix2d::Ones());
  kf.set_F((Eigen::Matrix2d() << 1, 1, 0, 1).finished());
  kf.set_B(Eigen::Matrix2d::Identity());
  kf.set_H(Eigen::Matrix2d::Identity());
  kf.set_Q(Eigen::Matrix2d::Identity());
  kf.set_R(Eigen::Matrix2d::Identity());
  std::vector<Models::ModelType> model_types{Models::ModelType::TEST_EMPTY_MODEL};
  std::shared_ptr<ModelInputGenerator> model_input_generator =
    std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator =
    std::make_shared<TransmissionProbabilityGenerator>();

  InteractingMultipleModelFilter imm;
  imm.setup(kf, model_types, model_input_generator, transmission_probability_generator);

  for (int i = 0; i < 10; i++) {
    imm.predict();
  }

  EXPECT_FLOAT_EQ(imm.get_state_estimate().x(), 11.0);
  EXPECT_FLOAT_EQ(imm.get_state_estimate().y(), 1.0);
}

TEST(InteractingMultipleModelFilter, update_ShouldReturnUpdate_WhenUpdateX100)
{
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{0.0, 0.0});
  kf.set_initial_p(Eigen::Matrix2d::Ones());
  kf.set_F((Eigen::Matrix2d() << 1, 1, 0, 1).finished());
  kf.set_B(Eigen::Matrix2d::Identity());
  kf.set_H(Eigen::Matrix2d::Identity());
  kf.set_Q(Eigen::Matrix2d::Identity());
  kf.set_R(Eigen::Matrix2d::Identity());
  std::vector<Models::ModelType> model_types{Models::ModelType::TEST_EMPTY_MODEL};
  std::shared_ptr<ModelInputGenerator> model_input_generator =
    std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator =
    std::make_shared<TransmissionProbabilityGenerator>();

  InteractingMultipleModelFilter imm;
  imm.setup(kf, model_types, model_input_generator, transmission_probability_generator);

  for (int i = 0; i < 10; i++) {
    imm.predict();  // Need to predict a bit to get P_hat to better represent the truth
  }

  for (int i = 0; i < 100; i++) {
    imm.update(Eigen::Vector2d{5.0, 10.0});
  }

  EXPECT_NEAR(imm.get_state_estimate().x(), 5.0, .1);
  EXPECT_NEAR(imm.get_state_estimate().y(), 10.0, .1);
}

TEST(InteractingMultipleModelFilter, clone_ShouldReturnIntial_WhenNothingChanges)
{
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{1.0, 2.0});
  std::vector<Models::ModelType> model_types{Models::ModelType::TEST_EMPTY_MODEL};
  std::shared_ptr<ModelInputGenerator> model_input_generator =
    std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator =
    std::make_shared<TransmissionProbabilityGenerator>();

  InteractingMultipleModelFilter imm;
  imm.setup(kf, model_types, model_input_generator, transmission_probability_generator);

  InteractingMultipleModelFilter cloned = imm.clone(Eigen::Vector2d{3.0, 4.0});

  EXPECT_FLOAT_EQ(imm.get_state_estimate().x(), 1.0);
  EXPECT_FLOAT_EQ(imm.get_state_estimate().y(), 2.0);
  EXPECT_FLOAT_EQ(cloned.get_state_estimate().x(), 3.0);
  EXPECT_FLOAT_EQ(cloned.get_state_estimate().y(), 4.0);
}

TEST(
  InteractingMultipleModelFilter,
  getPotentialMeasurementError_ShouldReturnDist_WhenNothingChanges)
{
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{1.0, 1.0});
  kf.set_initial_p(Eigen::Matrix2d::Ones());
  kf.set_F((Eigen::Matrix2d() << 1, 1, 0, 1).finished());
  kf.set_B(Eigen::Matrix2d::Identity());
  kf.set_H(Eigen::Matrix2d::Identity());
  kf.set_Q(Eigen::Matrix2d::Identity());
  kf.set_R(Eigen::Matrix2d::Identity());
  std::vector<Models::ModelType> model_types{Models::ModelType::TEST_EMPTY_MODEL};
  std::shared_ptr<ModelInputGenerator> model_input_generator =
    std::make_shared<ModelInputGenerator>();
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator =
    std::make_shared<TransmissionProbabilityGenerator>();

  InteractingMultipleModelFilter imm;
  imm.setup(kf, model_types, model_input_generator, transmission_probability_generator);

  EXPECT_FLOAT_EQ(imm.get_potential_measurement_error(Eigen::Vector2d{2.0, 1.0}), 0.0);
  EXPECT_FLOAT_EQ(imm.get_potential_measurement_error(Eigen::Vector2d{1.0, 1.0}), 1.0);
  EXPECT_FLOAT_EQ(imm.get_potential_measurement_error(Eigen::Vector2d{3.0, 1.0}), 1.0);
  EXPECT_FLOAT_EQ(imm.get_potential_measurement_error(Eigen::Vector2d{2.0, 2.0}), 1.0);
}
