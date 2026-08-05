#include "filtered_ball.hpp"

namespace ateam_super_vision {
FilteredBall::FilteredBall(const BallMeasurement & measurement)
{
  Eigen::VectorXd initial_state_xy(4);
    // We don't get a velocity input in the measurement itself,
    // so that starts at 0.
  initial_state_xy <<
    measurement.pos.x(),
    measurement.pos.y(),
    0,
    0;
    // This is in m, so initial covariance is 100 mm.
  Eigen::MatrixXd initial_error_covar = Eigen::MatrixXd::Identity(4, 4) * 1e-3;

    // We don't add any control inputs to the vision system, so the
    // control model maps a zero-length control vector onto the state.
  Eigen::MatrixXd control_model = Eigen::MatrixXd::Zero(4, 0);
  Eigen::MatrixXd measurement_model = PosMeasurementModel::measurementMatrix();
  Eigen::MatrixXd measurement_noise_covar = PosMeasurementModel::measurementNoiseCovar();
  Eigen::MatrixXd process_noise_covar = PosSystemModel::processNoiseCovar();

  posFilterXY.set_control_model(control_model);
  posFilterXY.set_measurement_model(measurement_model);
  posFilterXY.set_measurement_noise_covar(measurement_noise_covar);
  posFilterXY.set_process_noise_covar(process_noise_covar);

  posFilterXY.init(initial_state_xy, initial_error_covar);
  posXYEstimate = initial_state_xy;
}

void FilteredBall::update(const BallMeasurement & measurement)
{
    // Make sure this detection isn't crazy off from our previous ones
    // (unless our filter is still new/only has a few measurements)
  if (age < oldEnough) {
    ++age;
  }
  if (health < maxHealth) {
    health += 2;
  }
  bool is_new = age < oldEnough;
    // As long as its reasonable, update the Kalman Filter
  const std::chrono::time_point<std::chrono::steady_clock> now =
    std::chrono::steady_clock::now();
    // If it's been too long, don't use this message
  if (now - measurement.timestamp > update_threshold || is_new) {
    return;
  }
    // The state transition matrix (F) depends on the elapsed time, so it's
    // rebuilt every update with the latest dt.
  Eigen::MatrixXd state_transition_model = PosSystemModel::stateTransition(systemModelXY.stepDt());
  posFilterXY.set_state_transition_model(state_transition_model);

    // Predict state forward
    // Predict covariance forward
    // (All encompassed by the .predict() function)
  Eigen::VectorXd control_input(0);
  posFilterXY.predict(control_input);

    // Compute Kalman gain (contained in filter)
    // Update state estimate (contained in filter)
    // Update covariance estimate (contained in filter)
    // All encompassed by the .update() function
  Eigen::VectorXd z(2);
  z << measurement.pos.x(), measurement.pos.y();
  posFilterXY.update(z);

  posXYEstimate = posFilterXY.get_state_estimate();
}

ateam_msgs::msg::VisionStateBall FilteredBall::toMsg()
{
  ateam_msgs::msg::VisionStateBall ball_state_msg{};
  bool is_new = age < oldEnough;

  if (health > 0 && !is_new) {
        // NOTE: Does not contain acceleration info
    ball_state_msg.visible = true;
    ball_state_msg.pose.position.x = posXYEstimate.px();
    ball_state_msg.pose.position.y = posXYEstimate.py();
    ball_state_msg.twist.linear.x = posXYEstimate.vx();
    ball_state_msg.twist.linear.y = posXYEstimate.vy();
    --health;
  }
  return ball_state_msg;
}

bool FilteredBall::isHealthy() const
{
  return health > 0;
}
} // namespace ateam_super_vision