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

#include <vector>

TEST(KalmanFilter, getXHat_ShouldReturnIntial_WhenNoPredictOrUpdate)
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

TEST(KalmanFilter, getXHat_ShouldReturnPrediction_WhenPredictx10WithNoInput)
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

  for (int i = 0; i < 10; i++) {
    kf.predict(Eigen::Vector2d::Zero());
  }

  Eigen::VectorXd x_hat = kf.get_x_hat();

  EXPECT_DOUBLE_EQ(x_hat.x(), 21.0);
  EXPECT_DOUBLE_EQ(x_hat.y(), 2.0);
}

TEST(KalmanFilter, getXHat_ShouldReturnPrediction_WhenPredictx10WithInput)
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

  for (int i = 0; i < 10; i++) {
    kf.predict(Eigen::Vector2d{1, 0});
  }

  Eigen::VectorXd x_hat = kf.get_x_hat();

  EXPECT_DOUBLE_EQ(x_hat.x(), 31.0);
  EXPECT_DOUBLE_EQ(x_hat.y(), 2.0);
}

TEST(KalmanFilter, getXHat_ShouldReturnNearUpdate_WhenPredictx10ThenUpdatex10)
{
  KalmanFilter kf;
  kf.set_initial_x_hat(Eigen::Vector2d{1.0, 2.0});
  kf.set_initial_p(Eigen::Matrix2d::Ones());

  Eigen::Matrix2d F;
  F << 1, 1, 0, 1;
  Eigen::Matrix2d Q;
  Q << 1.0 / 4.0, 1.0 / 2.0, 1.0 / 2.0, 1.0;
  kf.set_F(F);
  kf.set_B(Eigen::Matrix2d::Identity());
  kf.set_H(Eigen::Matrix2d::Identity());
  kf.set_Q(Q);
  kf.set_R(Eigen::Matrix2d::Identity());

  for (int i = 0; i < 10; i++) {
    kf.predict(Eigen::Vector2d::Zero());
  }

  for (int i = 0; i < 10; i++) {
    kf.update(Eigen::Vector2d{1, 1});
  }

  Eigen::VectorXd x_hat = kf.get_x_hat();

  EXPECT_NEAR(x_hat.x(), 1.0, 0.1);
  EXPECT_NEAR(x_hat.y(), 1.0, 0.1);
}

TEST(KalmanFilter, getPotentialMeasurementError_ShouldReturnDistanceFromXHat_WhenXHatInitialized)
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

TEST(KalmanFilter, numeric_example)
{
  // Taken from here https://www.kalmanfilter.net/multiExamples.html
  KalmanFilter kf;

  double dt = 1;
  Eigen::Matrix<double, 6, 6> F;
  F << 1, dt, 0.5 * dt * dt, 0, 0, 0,
    0, 1, dt, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, dt, 0.5 * dt * dt,
    0, 0, 0, 0, 1, dt,
    0, 0, 0, 0, 0, 1;

  double sigma_alpha_squared = 0.15;
  Eigen::Matrix<double, 6, 6> Q;
  Q << dt * dt * dt * dt / 4, dt * dt * dt / 2, dt * dt / 2, 0, 0, 0,
    dt * dt * dt / 2, dt * dt, dt, 0, 0, 0,
    dt * dt / 2, dt, 1, 0, 0, 0,
    0, 0, 0, dt * dt * dt * dt / 4, dt * dt * dt / 2, dt * dt / 2,
    0, 0, 0, dt * dt * dt / 2, dt * dt, dt,
    0, 0, 0, dt * dt / 2, dt, 1;
  Q *= sigma_alpha_squared;

  double sigma_x_squared = 9;
  double sigma_y_squared = 9;
  Eigen::Matrix<double, 2, 2> R;
  R << sigma_x_squared, 0, 0, sigma_y_squared;

  Eigen::Matrix<double, 2, 6> H;
  H << 1, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0;

  Eigen::Matrix<double, 6, 1> x_init;
  x_init << 0, 0, 0, 0, 0, 0;

  Eigen::Matrix<double, 6, 6> P_init;
  P_init << 500, 0, 0, 0, 0, 0,
    0, 500, 0, 0, 0, 0,
    0, 0, 500, 0, 0, 0,
    0, 0, 0, 500, 0, 0,
    0, 0, 0, 0, 500, 0,
    0, 0, 0, 0, 0, 500;

  kf.set_initial_x_hat(x_init);
  kf.set_initial_p(P_init);
  kf.set_F(F);
  kf.set_Q(Q);
  kf.set_B(Eigen::Matrix<double, 6, 1>::Zero());
  kf.set_R(R);
  kf.set_H(H);

  kf.predict(Eigen::Matrix<double, 1, 1>::Zero());
  EXPECT_NEAR(kf.get_x_hat().norm(), 0.0, 1e-6);
  Eigen::Matrix<double, 6, 6> P0 = kf.get_P_hat();
  Eigen::Matrix<double, 6, 6> true_P0;
  true_P0 << 1125, 750, 250, 0, 0, 0,
    750, 1000, 500, 0, 0, 0,
    250, 500, 500, 0, 0, 0,
    0, 0, 0, 1125, 750, 250,
    0, 0, 0, 750, 1000, 500,
    0, 0, 0, 250, 500, 500;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      EXPECT_NEAR(P0(i, j), true_P0(i, j), 1);
    }
  }


  Eigen::Vector2d z1{-393.66, 300.4};
  kf.update(z1);

  Eigen::Matrix<double, 6, 1> x_hat1 = kf.get_x_hat();
  Eigen::Matrix<double, 6, 1> true_x_hat1;
  true_x_hat1 << -390.54, -260.36, -86.8, 298.02, 198.7, 66.23;

  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(x_hat1(i, 0), true_x_hat1(i, 0), 1);
  }

  Eigen::Matrix<double, 6, 6> P1 = kf.get_P_hat();
  Eigen::Matrix<double, 6, 6> true_P1;
  true_P1 << 8.93, 5.95, 2, 0, 0, 0,
    5.95, 504, 334.7, 0, 0, 0,
    2, 334.7, 444.9, 0, 0, 0,
    0, 0, 0, 8.93, 5.95, 2,
    0, 0, 0, 5.95, 504, 334.7,
    0, 0, 0, 2, 334.7, 444.9;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      EXPECT_NEAR(P1(i, j), true_P1(i, j), 1);
    }
  }

  kf.predict(Eigen::Matrix<double, 1, 1>::Zero());

  Eigen::Matrix<double, 6, 1> x_hat2 = kf.get_x_hat();
  Eigen::Matrix<double, 6, 1> true_x_hat2;
  true_x_hat2 << -694.3, -347.15, -86.8, 529.8, 264.9, 66.23;

  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(x_hat2(i, 0), true_x_hat2(i, 0), 1);
  }

  Eigen::Matrix<double, 6, 6> P2 = kf.get_P_hat();
  Eigen::Matrix<double, 6, 6> true_P2;
  true_P2 << 972, 1236, 559, 0, 0, 0,
    1236, 1618, 780, 0, 0, 0,
    559, 780, 445, 0, 0, 0,
    0, 0, 0, 972, 1236, 559,
    0, 0, 0, 1236, 1618, 780,
    0, 0, 0, 559, 780, 445;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      EXPECT_NEAR(P2(i, j), true_P2(i, j), 1);
    }
  }


  Eigen::Vector2d z2{-375.93, 301.78};
  kf.update(z2);

  Eigen::Matrix<double, 6, 1> x_hat3 = kf.get_x_hat();
  Eigen::Matrix<double, 6, 1> true_x_hat3;
  true_x_hat3 << -378.9, 53.8, 94.5, 303.9, -22.3, -63.6;

  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(x_hat3(i, 0), true_x_hat3(i, 0), 1);
  }

  Eigen::Matrix<double, 6, 6> P3 = kf.get_P_hat();
  Eigen::Matrix<double, 6, 6> true_P3;
  true_P3 << 8.92, 11.33, 5.13, 0, 0, 0,
    11.33, 61.1, 75.4, 0, 0, 0,
    5.13, 75.4, 126.5, 0, 0, 0,
    0, 0, 0, 8.92, 11.33, 5.13,
    0, 0, 0, 11.33, 61.1, 75.4,
    0, 0, 0, 5.13, 75.4, 126.5;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      EXPECT_NEAR(P3(i, j), true_P3(i, j), 1);
    }
  }

  kf.predict(Eigen::Matrix<double, 1, 1>::Zero());

  Eigen::Matrix<double, 6, 1> x_hat4 = kf.get_x_hat();
  Eigen::Matrix<double, 6, 1> true_x_hat4;
  true_x_hat4 << -277.8, 148.3, 94.5, 249.8, -85.9, -63.6;

  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(x_hat4(i, 0), true_x_hat4(i, 0), 1);
  }

  Eigen::Matrix<double, 6, 6> P4 = kf.get_P_hat();
  Eigen::Matrix<double, 6, 6> true_P4;
  true_P4 << 204.9, 254, 143.8, 0, 0, 0,
    254, 338.5, 202, 0, 0, 0,
    143.8, 202, 126.5, 0, 0, 0,
    0, 0, 0, 204.9, 254, 143.8,
    0, 0, 0, 254, 338.5, 202,
    0, 0, 0, 143.8, 202, 126.5;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      EXPECT_NEAR(P4(i, j), true_P4(i, j), 1);
    }
  }

  std::vector<Eigen::Vector2d> measurements;
  measurements.push_back(Eigen::Vector2d{-351.04, 295.1});
  measurements.push_back(Eigen::Vector2d{-328.96, 305.19});
  measurements.push_back(Eigen::Vector2d{-299.35, 301.06});
  measurements.push_back(Eigen::Vector2d{-245.89, 300});
  measurements.push_back(Eigen::Vector2d{-222.58, 303.57});
  measurements.push_back(Eigen::Vector2d{-198.03, 296.33});
  measurements.push_back(Eigen::Vector2d{-174.17, 297.65});
  measurements.push_back(Eigen::Vector2d{-146.32, 297.41});
  measurements.push_back(Eigen::Vector2d{-123.72, 299.61});
  measurements.push_back(Eigen::Vector2d{-103.49, 299.6});
  measurements.push_back(Eigen::Vector2d{-78.23, 302.39});
  measurements.push_back(Eigen::Vector2d{-52.63, 295.04});
  measurements.push_back(Eigen::Vector2d{-23.34, 300.09});
  measurements.push_back(Eigen::Vector2d{25.96, 294.72});
  measurements.push_back(Eigen::Vector2d{49.72, 298.61});
  measurements.push_back(Eigen::Vector2d{76.94, 294.64});
  measurements.push_back(Eigen::Vector2d{95.38, 284.88});
  measurements.push_back(Eigen::Vector2d{119.83, 272.82});
  measurements.push_back(Eigen::Vector2d{144.01, 264.93});
  measurements.push_back(Eigen::Vector2d{161.84, 251.46});
  measurements.push_back(Eigen::Vector2d{180.56, 241.27});
  measurements.push_back(Eigen::Vector2d{201.42, 222.98});
  measurements.push_back(Eigen::Vector2d{222.62, 203.73});
  measurements.push_back(Eigen::Vector2d{239.4, 184.1});
  measurements.push_back(Eigen::Vector2d{252.51, 166.12});
  measurements.push_back(Eigen::Vector2d{266.26, 138.71});
  measurements.push_back(Eigen::Vector2d{271.75, 119.71});
  measurements.push_back(Eigen::Vector2d{277.4, 100.41});
  measurements.push_back(Eigen::Vector2d{294.12, 79.76});
  measurements.push_back(Eigen::Vector2d{301.23, 50.62});
  measurements.push_back(Eigen::Vector2d{291.8, 32.99});
  measurements.push_back(Eigen::Vector2d{299.89, 2.14});

  for (const auto & measurement : measurements) {
    kf.update(measurement);
    kf.predict(Eigen::Matrix<double, 1, 1>::Zero());
  }

  Eigen::Matrix<double, 6, 1> x_hat6 = kf.get_x_hat();
  Eigen::Matrix<double, 6, 1> true_x_hat6;
  true_x_hat6 << 298.5, -1.65, -1.9, -22.5, -26.1, -0.64;

  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(x_hat6(i, 0), true_x_hat6(i, 0), 1.5);
  }

  Eigen::Matrix<double, 6, 6> P6 = kf.get_P_hat();
  Eigen::Matrix<double, 6, 6> true_P6;
  true_P6 << 11.25, 4.5, 0.9, 0, 0, 0,
    4.5, 2.4, 0.6, 0, 0, 0,
    0.9, 0.6, 0.2, 0, 0, 0,
    0, 0, 0, 11.25, 4.5, 0.9,
    0, 0, 0, 4.5, 2.4, 0.6,
    0, 0, 0, 0.9, 0.6, 0.2;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      EXPECT_NEAR(P6(i, j), true_P6(i, j), 5);
    }
  }
}
