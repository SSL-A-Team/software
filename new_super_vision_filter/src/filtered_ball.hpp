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

#ifndef FILTERED_BALL_HPP_ 
#define FILTERED_BALL_HPP_

#include <chrono>
#include <Eigen/Core>
#include <ssl_league_msgs/msg/vision_detection_ball.hpp>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include "kalman/ExtendedKalmanFilter.hpp"
#include "filter_types.hpp"

enum KickState {
    ROLLING,
    KICKED,
    CHIPPED
};

class FilteredBall {
    public:
        FilteredBall(ssl_league_msgs::msg::VisionDetectionBall &ball_detection_msg);

        void update(ssl_league_msgs::msg::VisionDetectionBall &ball_detection)

        ateam_msgs::msg::BallState toMsg();

        bool isHealthy() const;

    private:
        int age = 0;
        int oldEnough = 3;
        int maxHealth = 20;
        int health = 2;
        double maxDistance = -1.0;
        KickState currentKickState = ROLLING;
        std::chrono::milliseconds update_threshold{50};
        std::chrono::steady_clock::time_point timestamp; 
        std::chrono::steady_clock::time_point last_visible_timestamp;
        // Filter
        Kalman::ExtendedKalmanFilter<PosState> posFilterXY;
};

#endif // FILTERED_BALL_HPP_
