#include <Eigen/Dense>

// Standard kalman filter

class KalmanFilter {
public:
  void set_initial_x_hat(const Eigen::VectorXd & x_hat);
  void set_initial_p(const Eigen::MatrixXd & P);
  void set_F(const Eigen::MatrixXd & F);
  void set_B(const Eigen::MatrixXd & B);
  void set_H(const Eigen::MatrixXd & H);
  void set_Q(const Eigen::MatrixXd & Q);
  void set_R(const Eigen::MatrixXd & R);

  void predict(const Eigen::VectorXd & u);
  void update(const Eigen::VectorXd & z);

  Eigen::VectorXd get_x_hat() const;
  Eigen::VectorXd get_potential_measurement_error(const Eigen::VectorXd & z);

private:
  Eigen::MatrixXd F;
  Eigen::MatrixXd B;
  Eigen::MatrixXd H;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;

  Eigen::VectorXd x_hat;
  Eigen::MatrixXd P;
};