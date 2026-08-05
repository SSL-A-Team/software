#ifndef FILTER__KALMAN_FILTER_HPP_
#define FILTER__KALMAN_FILTER_HPP_

#include <Eigen/Core>

namespace ateam_super_vision {

class KalmanFilter {
    public:
        KalmanFilter();

        void init(Eigen::VectorXd &initial_state, Eigen::MatrixXd &initial_error_covar);

        void predict(Eigen::VectorXd &control_input);

        void update(Eigen::VectorXd &measurement);

        void set_process_noise_covar(Eigen::MatrixXd &covar_mat);

        void set_measurement_noise_covar(Eigen::MatrixXd &covar_mat);

        void set_state_transition_model(Eigen::MatrixXd &model_mat);

        void set_control_model(Eigen::MatrixXd &model_mat);

        void set_measurement_model(Eigen::MatrixXd &model_mat);

        const Eigen::VectorXd & get_state_estimate() const;

    private:
        // P matrix
        Eigen::MatrixXd error_covar;
        // Q matrix
        Eigen::MatrixXd process_noise_covar;
        // R matrix
        Eigen::MatrixXd measurement_noise_covar;
        // F matrix
        Eigen::MatrixXd state_transition_model;
        // B matrix
        Eigen::MatrixXd control_model;
        // H matrix
        Eigen::MatrixXd measurement_model;

        // x_hat
        Eigen::VectorXd state_estimate;
};

} // namespace ateam_super_vision

#endif // FILTER__KALMAN_FILTER_HPP_