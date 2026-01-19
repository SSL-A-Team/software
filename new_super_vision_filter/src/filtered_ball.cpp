#include "filtered_ball.hpp"

FilteredBall::FilteredBall(ssl_league_msgs::msg::VisionDetectionBall &ball_detection_msg){
    PosState initial_state_xy;
    initial_state_xy << 
        track.pos.px(),
        track.pos.py(),
        0,
        0;
    posFilterXY.init(initial_state_xy);
    Kalman::Matrix<double, 4, 4> xy_covariance;
    // This is in m, so initial covariance is 100 mm.
    // We don't get a velocity input in the measurement itself,
    // so that has a large initial uncertainty.
    xy_covariance    <<   1e-2, 0, 0, 0,
                        0, 1e-2, 0, 0,
                        0, 0, 1e3, 0,
                        0, 0, 0, 1e3;
    posFilterXY.setCovariance(xy_covariance);
}

void FilteredBall::update(BallTrack &track){
    // Make sure this detection isn't crazy off from our previous ones
    // (unless our filter is still new/only has a few measurements)
    ++age;
    bool is_new = age < oldEnough;
    // As long as its reasonable, update the Kalman Filter
    const std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    // If it's been too long, don't use this message
    if (now - track.timestamp > update_threshold || is_new) {
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
    posXYEstimate = posFilterXY.update(measurementModelXY, track.pos);
}

ateam_msgs::msg::BallState FilteredBall::toMsg(){
    ateam_msgs::msg::BallState ball_state_msg{};
    return ball_state_msg;
}