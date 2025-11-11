#include "filtered_robot.hpp"

#include <chrono>
#include <Eigen/Core>

FilteredRobot::FilteredRobot(ssl_league_msgs::msg::VisionDetectionRobot robot_detection_msg, ateam_common::TeamColor team_color)
    : posFilterXYW(Eigen::Vector3(robot_detection_msg->pose->position->x, robot_detection_msg->pose->position->y, robot_detection_msg->pose->orientation->w)),
    bot_id(robot_detection_msg->robot_id), height(robot_detection_msg->height * 1000), team(team_color) {}

void FilteredRobot::update(ssl_league_msgs::msg::VisionDetectionRobot robot_detection_msg) {
    // Make sure this detection isn't crazy off from our previous ones
    // (unless our filter is still new/only has a few measurements)
    bool is_new = age < oldEnough;
    // As long as its reasonable, update the Kalman Filter
}

ateam_msgs::msg::RobotState FilteredRobot::toMsg(){};