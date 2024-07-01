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

#ifndef FILTERS__KALMAN_FILTER_HPP_
#define FILTERS__KALMAN_FILTER_HPP_

#include <Eigen/Dense>

// Standard kalman filter

namespace ateam_vision_filter
{

class KalmanFilter
{
public:
  void set_initial_x_hat(const Eigen::VectorXd & x_hat);
  void set_initial_p(const Eigen::MatrixXd & P);
  void set_F(const Eigen::MatrixXd & F);
  void set_B(const Eigen::MatrixXd & B);
  void set_H(const Eigen::MatrixXd & H);
  void set_Q(const Eigen::MatrixXd & Q);
  void set_R(const Eigen::MatrixXd & R);
  void set_AngleMask(const Eigen::MatrixXd & angleMask);

  void predict(const Eigen::VectorXd & u);
  void update(const Eigen::VectorXd & z);

  Eigen::VectorXd get_x_hat() const;
  Eigen::MatrixXd get_P_hat() const;
  Eigen::VectorXd get_y() const;
  Eigen::MatrixXd get_estimated_gaussian_variance() const;
  Eigen::VectorXd get_potential_measurement_error(const Eigen::VectorXd & z);

private:
  Eigen::MatrixXd AngleMask;
  Eigen::MatrixXd F;
  // In many formulations, this is also represented as G
  Eigen::MatrixXd B;
  Eigen::MatrixXd H;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;

  Eigen::VectorXd x_hat;
  Eigen::MatrixXd P;
};

}  // namespace ateam_vision_filter

#endif  // FILTERS__KALMAN_FILTER_HPP_
