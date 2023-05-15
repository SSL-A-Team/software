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

CREATE_PARAM(double, "behavior_evaluator/", kRotationSpeed, 0.005);

BehaviorEvaluator::BehaviorEvaluator(BehaviorRealization & behavior_realization)
: behavior_realization(behavior_realization) {}

DirectedGraph<BehaviorGoal> BehaviorEvaluator::get_best_behaviors(const World & world)
{
  // Create a DirectedGraph of BehaviorPlans we are going to return
  DirectedGraph<BehaviorGoal> best_behaviors;
  // Get team color from world
  current_color_ = world.referee_info.our_team_color;
  // Get current command from world
  current_command_ = world.referee_info.running_command;

  // Check if we need to halt
  /*if (
      current_command_ == ateam_common::GameCommand::Halt) {
  }*/

  // Note: make sure we wait for a "start" before executing any of the plays
  // Have a setup function for whatever play is called to account for "stop" time
  // Check for special states

  // Check for kickoff
  // If our kickoff, basic kickoff play
  // If their kickoff, basic defensive play

  // Check for free kick
  // If their free kick, basic free kick defense
  // If our free kick, basic shoot

  // Check if there if we are in a penalty

  // Check if the ball is out of bounds

  // Else, implement either simple offense or simple defense

  /*DirectedGraph<BehaviorGoal> qual_goalie_and_shot;   
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
  }*/

  //
  // Add background behaviors
  //

  // std::vector<std::reference_wrapper<DirectedGraph<Behavior>>> possible_behaviors{
  //   three_one_touch_shot, direct_shot};
  // for (auto & behavior : possible_behaviors) {
  //   Behavior goalie{
  //     Behavior::Type::MoveToPoint,
  //     Behavior::Priority::Medium,
  //     MoveParam({0, 0})};  // Field::OurGoal.center();
  //   Behavior forward{
  //     Behavior::Type::MoveToPoint,
  //     Behavior::Priority::Low,
  //     MoveParam({10, 0})};  // Field::TheirGoal.center();
  //   Behavior left_defender{
  //     Behavior::Type::MoveToPoint,
  //     Behavior::Priority::Medium,
  //     MoveParam({5, 5})};
  //   Behavior right_defender{
  //     Behavior::Type::MoveToPoint,
  //     Behavior::Priority::Medium,
  //     MoveParam({5, -5})};

  //   behavior.get().add_node(goalie);
  //   behavior.get().add_node(forward);
  //   behavior.get().add_node(left_defender);
  //   behavior.get().add_node(right_defender);
  // }

  //
  // See how that combination of behaviors would be planned and executed
  //
  // DirectedGraph<BehaviorFeedback> three_one_touch_shot_feedback =
  //   behavior_realization.realize_behaviors(three_one_touch_shot, world);
  // DirectedGraph<BehaviorFeedback> direct_shot_feedback =
  //   behavior_realization.realize_behaviors(direct_shot, world);

  //
  // Choose main behavior
  //

  // choose direct shot because score chance is better or
  // maybe the total behavior completetion time is short
  // or maybe the other one can't be completed due to number of robots
  //DirectedGraph<BehaviorGoal> behavior_out = qual_goalie_and_shot;

  return behavior_out;
}
