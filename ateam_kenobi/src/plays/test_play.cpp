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
#include "robot_assignment.hpp"

namespace ateam_kenobi::plays
{
void TestPlay::reset(){

};
    
std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(World & world){
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

    std::vector<Robot> available_robots;
    for(const auto & maybe_robot : world.our_robots) {
        if(maybe_robot) {
            available_robots.push_back(maybe_robot.value());
        }
    }

    std::vector<ateam_geometry::Point> test_positions;
    test_positions.push_back(ateam_geometry::Point(0,0));

    const auto & robot_assignments = robot_assignment::assign(available_robots, test_positions);

    for(const auto [robot_id, pos_ind] : robot_assignments) {
        const auto & maybe_assigned_robot = world.our_robots.at(robot_id);
        if(!maybe_assigned_robot) { 
            // TODO Log this
            // Assigned non-available robot
            continue;
        }
        const auto & robot = maybe_assigned_robot.value();
        const auto & destination = test_positions.at(pos_ind);
        // TODO get path
        // TODO set path on controller
        // TODO get velocity command from controller
        maybe_motion_commands.at(robot_id) = {}; // TODO use the command from the controller
    }
    
    return maybe_motion_commands;
};
}  // namespace ateam_kenobi::plays
