#include "kalman_filter.hpp"

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

void KalmanFilter::set_Q(const Eigen::MatrixXd & Q)
{
  this->R = R;
}

void KalmanFilter::predict(const Eigen::VectorXd & u)
{
  x_hat = F * x_hat + B * u;
  P = F * P * F.tranpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd & z)
{
  Eigen::VectorXd y = z - H * x_hat;
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  Eigen::MatrixXd K = P * H.transpose() * S.inverse();
  x = x + K * y;
  P = (P.Ones() - K * H) * P;
}

Eigen::VectorXd KalmanFilter::get_x_hat() const {
  return x_hat;
}

double KalmanFilter::get_potential_measurement_error(const Eigen::VectorXd & z)
{
  return (z - H * x_hat).norm();
}