// Copyright 2024 A Team
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include <ateam_common/game_controller_listener.hpp>

#include "filtered_robot.hpp"

class FilteredRobotTest : public ::testing::Test
{
    protected:
        void SetUp() override
        {
            ssl_league_msgs::msg::VisionDetectionRobot robot_msg{};
            auto team = ateam_common::TeamColor::Blue; 
            FilteredRobot robot{
                robot_msg,
                team
            };
        }
}

TEST_F(RobotSetupTest, InitalState)
{
    // Check that initial state is valid
    return;
}

TEST_F(RobotSetupTest, InitialCovariance)
{
    // Check that initial covariance is valid
    return;
}

TEST_F(RobotUpdateTest, WaitUntilOldEnough)
{
    // Create valid timestamp and fake measurement
    // Try to update
    // Check that we didn't
    // Do that again until "oldEnough"
    // Now check that it has updated
    return;
}

TEST_F(RobotUpdateTest, UpdateIfTimestampValid)
{
    // Get initial estimate
    // Create valid timestamp and fake measurement
    // Update the filter
    // Check that the current estimate is different
    return;
}

TEST_F(RobotUpdateTest, DontUpdateIfTimestampInvalid)
{
    // Get initial estimate
    // Create invalid timestamp and fake measurement
    // Attempt to update the filter
    // Check that the current estimate is not different
    return;
}