#include "filtered_robot.hpp"
// #include "kalman/Types.hpp"
#include "filter_types.hpp"

#include <chrono>
#include <Eigen/Core>

FilteredRobot::FilteredRobot(ssl_league_msgs::msg::VisionDetectionRobot robot_detection_msg, ateam_common::TeamColor team_color)
    : posFilterXY(), posFilterW(), bot_id(robot_detection_msg->robot_id), height(robot_detection_msg->height * 1000), team(team_color) {
        // TODO (Christian) - Might need to change the below to use our state/measurement types
        // Initialize XY KF
        Kalman::Vector<4, double> initial_state_xy = { 
            robot_detection_msg->pose->position->x,
            robot_detection_msg->pose->position->y,
            0,
            0
        };
        posFilterXY.init(initial_state_xy);
        Kalman::Matrix<4, 4, double> xy_covariance <<   1e-2, 0, 0, 0,
                                                        0, 1e-2, 0, 0,
                                                        0, 0, 1e3, 0,
                                                        0, 0, 0, 1e3;
        posFilterXY.setCovariance(xy_covariance);

        // Initialize angular KF
        /*
            State vector is simply
            w_pos,
            w_vel
        */
        Kalman::Vector<2, double> initial_state_w = {
            robot_detection_msg->pose->orientation->w,
            0
        };
        posFilterW.init(initial_state_w);
        /*
            Initial covariance is approx 2 deg. for pos,
            10 deg. for vel
        */
        Kalman::Matrix<2, 2, double> w_covariance <<    (2*M_PI/180.0)^2, 0,
                                                        0, (10*M_PI/180.0)^2;
        posFilterW.setCovariance(w_covariance);
    }

void FilteredRobot::update(ssl_league_msgs::msg::VisionDetectionRobot robot_detection_msg) {
    // Make sure this detection isn't crazy off from our previous ones
    // (unless our filter is still new/only has a few measurements)
    bool is_new = age < oldEnough;
    // As long as its reasonable, update the Kalman Filter
}

ateam_msgs::msg::RobotState FilteredRobot::toMsg(){};