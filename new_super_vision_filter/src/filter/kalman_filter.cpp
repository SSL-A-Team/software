#include "filter/kalman_filter.hpp"

namespace ateam_super_vision {
    KalmanFilter::KalmanFilter() {}

    void KalmanFilter::init(Eigen::VectorXd & initial_state, Eigen::MatrixXd & initial_error_covar) {
        state_estimate = initial_state;
        error_covar = initial_error_covar;
    }

    void KalmanFilter::predict(Eigen::VectorXd & control_input) {
        state_estimate = state_transition_model * state_estimate + control_model * control_input;
        error_covar = state_transition_model * error_covar * state_transition_model.transpose() + process_noise_covar;
    }

    void KalmanFilter::update(Eigen::VectorXd &measurement) {
        // NOTE: This function expects you will normalize angles first outside of this function
        // unlike in Joe's implementation.
        Eigen::VectorXd predicted_measurement = measurement_model * state_estimate;
        Eigen::VectorXd measure_residual = measurement - predicted_measurement;

        // S matrix
        Eigen::MatrixXd innovation_covar = measurement_model * error_covar
            * measurement_model.transpose() + measurement_noise_covar;
        // K matrix
        Eigen::MatrixXd kalman_gain =  error_covar * measurement_model.transpose() * innovation_covar.inverse();

        state_estimate = state_estimate + kalman_gain * measure_residual;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(error_covar.rows(), error_covar.cols());
        // Also have to update the error covariance here
        error_covar = (I - kalman_gain * measurement_model) * error_covar *
            (I - kalman_gain * measurement_model).transpose() +
            (kalman_gain * measurement_noise_covar * kalman_gain.transpose());
    }

    void KalmanFilter::set_process_noise_covar(Eigen::MatrixXd &covar_mat) {
        this->process_noise_covar = covar_mat;
    }

    void KalmanFilter::set_measurement_noise_covar(Eigen::MatrixXd &covar_mat) {
        this->measurement_noise_covar = covar_mat;
    }

    void KalmanFilter::set_state_transition_model(Eigen::MatrixXd &model_mat) {
        this->state_transition_model = model_mat;
    }

    void KalmanFilter::set_control_model(Eigen::MatrixXd &model_mat) {
        this->control_model = model_mat;
    }

    void KalmanFilter::set_measurement_model(Eigen::MatrixXd &model_mat) {
        this->measurement_model = model_mat;
    }
}

