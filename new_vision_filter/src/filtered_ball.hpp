#include <chrono>
#include <Eigen/Core>

enum KickState {
    ROLLING,
    KICKED,
    CHIPPED
};

class FilteredBall {
    private:
        KickState currentKickState= ROLLING;
        std::chrono::time_point timestamp; 
        std::chrono::time_point last_visible_timestamp;
        // TODO: Add unit labels/info 
        Eigen::Vector3<double> pos;
        Eigen::Vector3<double> vel;
        Eigen::Vector3<double> acc;
        Eigen::Vector2<double> spin;
}