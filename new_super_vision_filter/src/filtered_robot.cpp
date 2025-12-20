#include "filtered_robot.hpp"
// #include "kalman/Types.hpp"
#include "filter_types.hpp"

#include <chrono>
#include <Eigen/Core>

// See https://thekalmanfilter.com/extended-kalman-filter-python-example/
// or https://thekalmanfilter.com/kalman-filter-explained-simply/
// OR https://github.com/mherb/kalman/blob/master/examples/Robot1/main.cpp

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
    const std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    // If it's been too long, don't use this message
    if (now - timestamp > update_threshold) {
        timestamp = now;
        return;
    }

    // Predict state forward
    // Predict covariance forward
    // (All encompassed by the .predict() function)
    auto xy_pred = posFilterXY.predict();
    auto w_pred = posFilterW.predict();
    // TODO - create the pos variable (should just be a vector) from our detection message
    PosMeasurement xy_measurement = measurementModelXY.h(pos);
    AngleMeasurement w_measurement = measurementModelW.h(pos);
    // Update the Jacobian matrix (contained in filter)
    // Compute Kalman gain (contained in filter)
    // Update state estimate (returned)
    // Update covariance estimate (contained in filter)
    // All encompassed by the .update() function
    auto xy_updated = posFilterXY.update(measurementModelXY, xy_measurement)
    auto w_updated = posFilterW.update(measurementModelW, w_measurement)
    // Update our timestamp for next time
    timestamp = now;
}

ateam_msgs::msg::RobotState FilteredRobot::toMsg(){};