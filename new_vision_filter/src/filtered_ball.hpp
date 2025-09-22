#include <chrono>
#include <Eigen/Core>
#include <ssl_league_msgs/msg/vision_detection_ball.hpp>
#include "kalman/ExtendedKalmanFilter.hpp"

enum KickState {
    ROLLING,
    KICKED,
    CHIPPED
};

class FilteredBall {
    FilteredBall::FilteredBall(ssl_league_msgs::msg::VisionDetectionBall vision_ball);

    private:
        KickState currentKickState = ROLLING;
        std::chrono::steady_clock::time_point timestamp; 
        std::chrono::steady_clock::time_point last_visible_timestamp;
        // TODO: Add unit labels/info 
        Eigen::Vector3<double> pos;
        Eigen::Vector3<double> vel;
        Eigen::Vector3<double> acc;
        Eigen::Vector2<double> spin;
        // Filter
        ExtendedKalmanFilter<Kalman::Vector2::double> posFilterXY;
}