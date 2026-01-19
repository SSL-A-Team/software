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
#ifndef FILTERED_ROBOT_HPP_ 
#define FILTERED_ROBOT_HPP_ 

#include "kalman/ExtendedKalmanFilter.hpp"
#include "filter_types.hpp"

#include <chrono>
#include <Eigen/Core>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_msgs/msg/vision_state_robot.hpp>

class FilteredRobot {
    public:
        FilteredRobot(RobotTrack &track, ateam_common::TeamColor team_color);

        void update(ssl_league_msgs::msg::VisionDetectionRobot robot_detection);

        ateam_msgs::msg::RobotState toMsg();

        int getId() const;

    private:
        int bot_id;
        ateam_common::TeamColor team;
        int age = 0;
        int oldEnough = 3;
        int maxHealth = 20;
        double maxDistance = -1.0;
        std::chrono::milliseconds update_threshold{50};
        std::chrono::time_point<std::chrono::steady_clock> timestamp; 
        std::chrono::time_point<std::chrono::steady_clock> last_visible_timestamp;
        // double height; // in m
        // Use the below filtered values when getting X/Y/w (theta) and
        // velocities
        // X, Y
        Kalman::ExtendedKalmanFilter<PosState> posFilterXY;
        PosSystemModel systemModelXY;
        PosState posXYEstimate{};
        // Theta
        Kalman::ExtendedKalmanFilter<AngleState> posFilterW;
        AngleSystemModel systemModelW;
        AngleState posWEstimate{};
};

#endif  // FILTERED_ROBOT_HPP_