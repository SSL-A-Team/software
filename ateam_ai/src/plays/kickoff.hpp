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

#ifndef PLAYS__KICKOFF_HPP_
#define PLAYS__KICKOFF_HPP_

#include <vector>

#include "util/directed_graph.hpp"
#include "types/behavior_goal.hpp"
#include "types/world.hpp"
#include "plays/defense.hpp"

DirectedGraph<BehaviorGoal> setup_our_kickoff(
  const World & world)
{
  DirectedGraph<BehaviorGoal> our_kickoff;
  // TODO(Christian): Replace the below with shared diameter constant
  double robot_diameter = 0.18;

  // Have the kicker robot go to the edge of the center circle to prepare for kick
  BehaviorGoal kicker_setup {
    BehaviorGoal::Type::MoveToPoint,
    BehaviorGoal::Priority::Required,
    // We must be outside the center circle on OUR side
    // Its diameter is 1m
    MoveParam(Eigen::Vector2d{-0.55 + robot_diameter, 0})
  };

  our_kickoff.add_node(kicker_setup);

  // Add the goalie
  our_kickoff.add_node(get_goalie_behavior_goal(world));

  // Make sure the other three robots are on our side
  // For now, setting these to hard coded locations
  // unless we want to determine
  // a good way to generate better ones...
  BehaviorGoal right_striker {
    BehaviorGoal::Type::MoveToPoint,
    BehaviorGoal::Priority::Required,
    MoveParam(Eigen::Vector2d{-0.3, -1.5})
  };
  our_kickoff.add_node(right_striker);

  BehaviorGoal left_striker {
    BehaviorGoal::Type::MoveToPoint,
    BehaviorGoal::Priority::Required,
    MoveParam(Eigen::Vector2d{-0.3, 1.5})
  };
  our_kickoff.add_node(left_striker);

  BehaviorGoal back_defense {
    BehaviorGoal::Type::MoveToPoint,
    BehaviorGoal::Priority::Required,
    MoveParam(Eigen::Vector2d{-2, 0})
  };
  our_kickoff.add_node(back_defense);

  return our_kickoff;
}
#endif  // PLAYS__KICKOFF_HPP_
