#include <chrono>
#include <Eigen/Core>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>

class FilteredRobot {
    FilteredRobot::FilteredRobot()

    private:
        int bot_id;
        // TODO: use a global enum for yellow/blue?
        int team; // for now 0 = us, 1 = not us
        std::chrono::time_point timestamp; 
        std::chrono::time_point last_visible_timestamp;
        // TODO: what units to use?
        Eigen::Vector3<double> pos;
        Eigen::Vector3<double> vel;
        Eigen::Vector3<double> acc;
        double orientation; // rads
        double angularVel; // rads/sec
        // X, Y, Angle
        ExtendedKalmanFilter<Kalman::Vector3::double> posFilterXYW;
}