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

#include "filters/kalman_filter.hpp"

void KalmanFilter::set_initial_x_hat(const Eigen::VectorXd & x_hat)
{
  this->x_hat = x_hat;
}

void KalmanFilter::set_initial_p(const Eigen::MatrixXd & P)
{
  this->P = P;
}

void KalmanFilter::set_F(const Eigen::MatrixXd & F)
{
  this->F = F;
}

void KalmanFilter::set_B(const Eigen::MatrixXd & B)
{
  this->B = B;
}

void KalmanFilter::set_H(const Eigen::MatrixXd & H)
{
  this->H = H;
}

void KalmanFilter::set_R(const Eigen::MatrixXd & R)
{
  this->R = R;
}

void KalmanFilter::set_Q(const Eigen::MatrixXd & Q)
{
  this->Q = Q;
}

void KalmanFilter::predict(const Eigen::VectorXd & u)
{
  x_hat = F * x_hat + B * u;
  P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd & z)
{
  Eigen::VectorXd y = z - H * x_hat;
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  Eigen::MatrixXd K = P * H.transpose() * S.inverse();
  x_hat = x_hat + K * y;
  P = P;  // TODO(jneiger): (P.Ones() - K * H) * P;
}

Eigen::VectorXd KalmanFilter::get_x_hat() const
{
  return x_hat;
}

Eigen::VectorXd KalmanFilter::get_y() const
{
  return H * x_hat;
}

Eigen::VectorXd KalmanFilter::get_estimated_gaussian_variance() const
{
  return H * P * H.transpose() + R;
}

Eigen::VectorXd KalmanFilter::get_potential_measurement_error(const Eigen::VectorXd & z)
{
  return z - H * x_hat;
}
