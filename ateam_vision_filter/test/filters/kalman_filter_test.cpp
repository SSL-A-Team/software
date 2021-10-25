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

#include "../src/filters/kalman_filter.hpp"

#include <gtest/gtest.h>

TEST(kalman_filter, getXHat_ShouldReturnIntial_WhenNoPredictOrUpdate)
{
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{1.0, 2.0});
  kf.set_initial_p(Eigen::Matrix2d::Ones());
  kf.set_F(Eigen::Matrix2d::Identity());
  kf.set_B(Eigen::Matrix2d::Identity());
  kf.set_H(Eigen::Matrix2d::Identity());
  kf.set_Q(Eigen::Matrix2d::Identity());
  kf.set_R(Eigen::Matrix2d::Identity());

  Eigen::VectorXd x_hat = kf.get_x_hat();

  EXPECT_DOUBLE_EQ(x_hat.x(), 1.0);
  EXPECT_DOUBLE_EQ(x_hat.y(), 2.0);
}

TEST(kalman_filter, getXHat_ShouldReturnPrediction_WhenPredictx10WithNoInput)
{
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{1.0, 2.0});
  kf.set_initial_p(Eigen::Matrix2d::Ones());

  Eigen::Matrix2d F;
  F << 1, 1, 0, 1;
  kf.set_F(F);
  kf.set_B(Eigen::Matrix2d::Identity());
  kf.set_H(Eigen::Matrix2d::Identity());
  kf.set_Q(Eigen::Matrix2d::Identity());
  kf.set_R(Eigen::Matrix2d::Identity());

  for (int i = 0; i < 10; i++)
  {
    kf.predict(Eigen::Vector2d::Zero());
  }

  Eigen::VectorXd x_hat = kf.get_x_hat();

  EXPECT_DOUBLE_EQ(x_hat.x(), 21.0);
  EXPECT_DOUBLE_EQ(x_hat.y(), 2.0);
}

TEST(kalman_filter, getXHat_ShouldReturnPrediction_WhenPredictx10WithInput)
{
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{1.0, 2.0});
  kf.set_initial_p(Eigen::Matrix2d::Ones());

  Eigen::Matrix2d F;
  F << 1, 1, 0, 1;
  kf.set_F(F);
  kf.set_B(Eigen::Matrix2d::Identity());
  kf.set_H(Eigen::Matrix2d::Identity());
  kf.set_Q(Eigen::Matrix2d::Identity());
  kf.set_R(Eigen::Matrix2d::Identity());

  for (int i = 0; i < 10; i++)
  {
    kf.predict(Eigen::Vector2d{1, 0});
  }

  Eigen::VectorXd x_hat = kf.get_x_hat();

  EXPECT_DOUBLE_EQ(x_hat.x(), 31.0);
  EXPECT_DOUBLE_EQ(x_hat.y(), 2.0);
}

TEST(kalman_filter, getXHat_ShouldReturnNearUpdate_WhenPredictx10ThenUpdatex10)
{
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{1.0, 2.0});
  kf.set_initial_p(Eigen::Matrix2d::Ones());

  Eigen::Matrix2d F;
  F << 1, 1, 0, 1;
  kf.set_F(F);
  kf.set_B(Eigen::Matrix2d::Identity());
  kf.set_H(Eigen::Matrix2d::Identity());
  kf.set_Q(Eigen::Matrix2d::Identity());
  kf.set_R(Eigen::Matrix2d::Identity());

  for (int i = 0; i < 10; i++)
  {
    kf.predict(Eigen::Vector2d::Zero());
  }

  for (int i = 0; i < 10; i++)
  {
    kf.update(Eigen::Vector2d{1, 1});
  }

  Eigen::VectorXd x_hat = kf.get_x_hat();

  EXPECT_NEAR(x_hat.x(), 1.0, 1e-4);
  EXPECT_NEAR(x_hat.y(), 1.0, 1e-4);
}

TEST(kalman_filter, getPotentialMeasurementError_ShouldReturnDistanceFromXHat_WhenXHatInitialized)
{
  Eigen::Vector2d x_hat_init{1.0, 2.0};

  KalmanFilter kf;
  kf.set_initial_x_hat(x_hat_init);
  kf.set_initial_p(Eigen::Matrix2d::Ones());
  kf.set_F(Eigen::Matrix2d::Identity());
  kf.set_B(Eigen::Matrix2d::Identity());
  kf.set_H(Eigen::Matrix2d::Identity());
  kf.set_Q(Eigen::Matrix2d::Identity());
  kf.set_R(Eigen::Matrix2d::Identity());

  Eigen::Vector2d potential_measurement{11.0, 2.0};
  Eigen::VectorXd measurement_error = kf.get_potential_measurement_error(potential_measurement);

  EXPECT_NEAR(measurement_error.x(), (potential_measurement - x_hat_init).x(), 1e-4);
  EXPECT_NEAR(measurement_error.y(), (potential_measurement - x_hat_init).y(), 1e-4);
}