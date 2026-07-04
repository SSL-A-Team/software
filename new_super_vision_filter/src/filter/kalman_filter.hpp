#ifndef FILTER__KALMAN_FILTER_HPP_
#define FILTER__KALMAN_FILTER_HPP_

#include <Eigen/Core>

namespace ateam_super_vision {

class KalmanFilter {
    public:
        KalmanFilter();

        init(initial_state);
        
        predict(control_input);

        update(measurement);

        set_process_noise_covar(covar_mat);

        set_measurement_noise_covar(covar_mat);

    private:
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
        Eigen::MatrixXd state_estimate;
}

} // namespace ateam_super_vision

#endif // FILTER__KALMAN_FILTER_HPP_