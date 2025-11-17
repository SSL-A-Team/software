#ifndef FILTERED_ROBOT_HPP_ 
#define FILTERED_ROBOT_HPP_ 

#include "kalman/ExtendedKalmanFilter.hpp"
#include "filter_types.hpp"

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
        double height; // in m
        // Use the below filtered values when getting X/Y/w (theta) and
        // velocities
        // X, Y
        Kalman::ExtendedKalmanFilter<PosState> posFilterXY;
        // Theta
        Kalman::ExtendedKalmanFilter<AngleState> posFilterW;
}

#endif  // FILTERED_ROBOT_HPP_