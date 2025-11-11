#ifndef FILTERED_ROBOT_HPP_ 
#define FILTERED_ROBOT_HPP_ 

#include <chrono>
#include <Eigen/Core>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include <ateam_common/game_controller_listener.hpp>

class FilteredRobot {
    FilteredRobot::FilteredRobot(ssl_league_msgs::msg::VisionDetectionRobot robot_detection_msg, ateam_common::TeamColor team_color);

    void update(ssl_league_msgs::msg::VisionDetectionRobot robot_detection);

    ateam_msgs::msg::RobotState toMsg();

    private:
        int bot_id;
        ateam_common::TeamColor team;
        int age;
        int oldEnough = 3;
        int maxHealth = 20;
        double maxDistance = -1.0;
        std::chrono::time_point timestamp; 
        std::chrono::time_point last_visible_timestamp;
        // Unless otherwise stated, below is in m and s
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