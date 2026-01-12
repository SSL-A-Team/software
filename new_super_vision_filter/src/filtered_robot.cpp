// Copyright 2025 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "filtered_robot.hpp"
#include "filter_types.hpp"
#include "measurements/robot_track.hpp"

#include <cmath>
#include <chrono>
#include <Eigen/Core>

// See https://thekalmanfilter.com/extended-kalman-filter-python-example/
// or https://thekalmanfilter.com/kalman-filter-explained-simply/
// OR https://github.com/mherb/kalman/blob/master/examples/Robot1/main.cpp

FilteredRobot::FilteredRobot(RobotTrack track, ateam_common::TeamColor team_color)
    : posFilterXY(), posFilterW(), bot_id(track.robot_id), team(team_color) {
        // TODO (Christian) - Might need to change the below to use our state/measurement types
        // Initialize XY KF
        PosState initial_state_xy;
        initial_state_xy << 
            track.pos.px(),
            track.pos.py(),
            0,
            0;
        posFilterXY.init(initial_state_xy);
        Kalman::Matrix<double, 4, 4> xy_covariance;
        xy_covariance    <<   1e-2, 0, 0, 0,
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
        AngleState initial_state_w;
        initial_state_w <<
            track.angle.pw(),
            0;
        posFilterW.init(initial_state_w);
        /*
            Initial covariance is approx 2 deg. for pos,
            10 deg. for vel
        */
        Kalman::Matrix<double, 2, 2> w_covariance;
        w_covariance <<    std::pow(2*M_PI/180.0, 2), 0,
                            0, std::pow(10*M_PI/180.0, 2);
        posFilterW.setCovariance(w_covariance);
    }

void FilteredRobot::update(RobotTrack &track) {
    // Make sure this detection isn't crazy off from our previous ones
    // (unless our filter is still new/only has a few measurements)
    ++age;
    bool is_new = age < oldEnough;
    // As long as its reasonable, update the Kalman Filter
    // TODO (Christian) - do this for each current track in the queue
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
    auto w_pred = posFilterW.predict(systemModelW);
    // Update the Jacobian matrix (contained in filter)
    // Compute Kalman gain (contained in filter)
    // Update state estimate (returned)
    // Update covariance estimate (contained in filter)
    // All encompassed by the .update() function
    posXYEstimate = posFilterXY.update(measurementModelXY, track.pos);
    posWEstimate = posFilterW.update(measurementModelW, track.angle);
}

ateam_msgs::msg::RobotState FilteredRobot::toMsg(){
    return ateam_msgs::msg::RobotState{};
};

int FilteredRobot::getId() const {
    return bot_id;
}