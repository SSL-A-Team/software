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

#include "dag_generation/stop.hpp"
#include "types/world.hpp"
#include "types/behavior_goal.hpp"

#include <Eigen/Dense>

DirectedGraph<BehaviorGoal> generate_stop(const World & world) {
    DirectedGraph<BehaviorGoal> stop_graph;

    for (std::size_t id = 0; id < world.our_robots.size(); id++) {
        // Get the current position
        const Eigen::Vector2d robot_position_ = world.our_robots.at(id).value().pos;

        // Check if this robot is within 0.5 m of the ball
        // Get the distance of a line defined by the current position of the robot and the ball
        // Is it less than 0.5 + the radius of the robot?

        // If so, generate a close trajectory to be away from the ball (in the direction the robot
        // is currently facing, or as close as possible if we are facing the ball)

        // Else, tell it to go to the current position
        BehaviorGoal stop {
            BehaviorGoal::Type::MoveToPoint,
            BehaviorGoal::Priority::Required,
            MoveParam(Eigen::Vector2d{robot_position_.x(), robot_position_.y()})
        };
        stop_graph.add_node(stop);
    }
    return stop_graph;
}