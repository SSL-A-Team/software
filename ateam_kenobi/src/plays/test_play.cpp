// Copyright 2023 A Team
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

#include "test_play.hpp"
#include "ateam_geometry/types.hpp"
#include "types/world.hpp"

TestPlay::TestPlay(World & world) : {
    this -> reset(world);
};

std::array<RobotMotionCommand, 16> reset(World & world){
    this -> current_frame = 0;
    this -> maybe_robot_trajectories_ = new std::array<std::optional<std::vector<ateam_geometry::Point>>, 16>;

    std::array<RobotMotionCommand, 16> maybe_motion_commands;

    std::vector<ateam_geometry::Point> test_position;
    test_position.push_back(ateam_geometry::Point(0,0));

    // Tell robot 0 to go to goal 0
    std::map<size_t, size_t> robots_to_points = {{0, 0}}

    // get a trajectory
    for (auto robot_to_point : robots_to_points){
        auto end_point = test_vector[robot_to_point.second];
        // generate trajectory here
        std::vector<ateam_geometry::Point> test_trajectory;
        maybe_robot_trajectories_[robot_to_point.first] = test_trajectory;
    }
    // return the first motion command for each robot
    for (size_t i = 0; i < maybe_robot_trajectories_.size(); ++i){
        if (!trajectory.has_value()){
            continue;
        }
        maybe_motion_commands[i] = RobotMotionCommand();
    }
    return maybe_motion_commands;
};
    
std::array<RobotMotionCommand, 16> runFrame(World & world){
    // For each trajectory that exists,
    // Get the next motion command along the trajectory
    // Given the world state
};