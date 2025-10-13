#ifndef FILTERED_ROBOT_HPP_ 
#define FILTERED_ROBOT_HPP_ 

#include <chrono>
#include <Eigen/Core>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>

class FilteredRobot {
    FilteredRobot::FilteredRobot(ssl_league_msgs::msg::VisionDetectionRobot robot_detection_msg, int team);

    void update(ssl_league_msgs::msg::VisionDetectionRobot robot_detection);

    private:
        int bot_id;
        // TODO: use a global enum for yellow/blue?
        int team; // for now 0 = us, 1 = not us
        int age;
        int oldEnough = 3;
        int maxHealth = 20;
        double maxDistance = -1.0;
        std::chrono::time_point timestamp; 
        std::chrono::time_point last_visible_timestamp;
        // Unless otherwise stated, below is in m
        Eigen::Vector3<double> pos;
        Eigen::Vector3<double> vel;
        Eigen::Vector3<double> acc;
        double orientation; // rads
        double angularVel; // rads/sec
        double height; // in m
        // X, Y, Angle
        ExtendedKalmanFilter<Kalman::Vector3::double> posFilterXYW;
}

#endif  // FILTERED_ROBOT_HPP_