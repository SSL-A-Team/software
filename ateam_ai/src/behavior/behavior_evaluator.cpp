// Copyright 2021 A Team
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

#include "behavior/behavior_evaluator.hpp"

#include <functional>
#include <vector>
#include <cmath>

#include <ateam_common/parameters.hpp>
#include <ateam_common/game_state_listener.hpp>
#include <ateam_common/team_color_listener.hpp>

CREATE_PARAM(double, "behavior_evaluator/", kRotationSpeed, 0.005);

BehaviorEvaluator::BehaviorEvaluator(BehaviorRealization & behavior_realization)
: behavior_realization(behavior_realization) {}

DirectedGraph<BehaviorGoal> BehaviorEvaluator::get_best_behaviors(const World & world)
{
  //
  // Qual Video Behaviors
  //
  DirectedGraph<BehaviorGoal> qual_goalie_and_shot;
  double ball_y = 0;
  if (world.get_unique_ball().has_value()) {
    ball_y = world.get_unique_ball().value().pos.y();
  }
  BehaviorGoal goalie{
    BehaviorGoal::Type::MoveToPoint,
    BehaviorGoal::Priority::Required,
    MoveParam(Eigen::Vector2d{-5, ball_y})};
  qual_goalie_and_shot.add_node(goalie);

  BehaviorGoal kicker{
    BehaviorGoal::Type::MovingKick,
    BehaviorGoal::Priority::Required,
    MoveParam(Eigen::Vector2d{0, 1})};
  qual_goalie_and_shot.add_node(kicker);

  for (int i = 2; i < 16; i++) {
    BehaviorGoal move{
      BehaviorGoal::Type::MoveToPoint,
      BehaviorGoal::Priority::Required,
      MoveParam(Eigen::Vector2d{i / -2.0, 4})};
  qual_goalie_and_shot.add_node(move);

  DirectedGraph<BehaviorGoal> behavior_out = qual_goalie_and_shot;
  return behavior_out;
  }
}