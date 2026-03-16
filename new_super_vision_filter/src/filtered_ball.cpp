#include "filtered_ball.hpp"

FilteredBall::FilteredBall(const BallMeasurement & measurement)
{
  PosState initial_state_xy;
  initial_state_xy <<
    measurement.pos.x(),
    measurement.pos.y(),
    0,
    0;
  posFilterXY.init(initial_state_xy);
  Kalman::Matrix<double, 4, 4> xy_covariance;
    // This is in m, so initial covariance is 100 mm.
    // We don't get a velocity input in the measurement itself,
    // so that has a large initial uncertainty.
  xy_covariance << 1e-2, 0, 0, 0,
    0, 1e-2, 0, 0,
    0, 0, 1e3, 0,
    0, 0, 0, 1e3;
  posFilterXY.setCovariance(xy_covariance);
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
    // Predict state forward
    // Predict covariance forward
    // (All encompassed by the .predict() function)
  auto xy_pred = posFilterXY.predict(systemModelXY);
    // Update the Jacobian matrix (contained in filter)
    // Compute Kalman gain (contained in filter)
    // Update state estimate (returned)
    // Update covariance estimate (contained in filter)
    // All encompassed by the .update() function
  posXYEstimate = posFilterXY.update(measurementModelXY, measurement.pos);
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
