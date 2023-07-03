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

#include "behavior/behavior_evaluator.hpp"

#include <functional>
#include <vector>
#include <cmath>

#include <ateam_common/parameters.hpp>
#include "plays/defense.hpp"

CREATE_PARAM(double, "behavior_evaluator/", kRotationSpeed, 0.005);

BehaviorEvaluator::BehaviorEvaluator(BehaviorRealization & behavior_realization)
: behavior_realization(behavior_realization) {}

DirectedGraph<BehaviorGoal> BehaviorEvaluator::get_best_behaviors(const World & world)
{
  //
  // Do preprocessing on world state to get important metrics like possession
  //

  //
  // Setup different behavior options
  //


  // DirectedGraph<Behavior> three_one_touch_shot;
  // Behavior initial_pass_start{
  //   Behavior::Type::MovingKick,
  //   Behavior::Priority::Required,
  //   KickParam({0, 0})};
  // Behavior first_receiver{
  //   Behavior::Type::OneTouchReceiveKick,
  //   Behavior::Priority::Required,
  //   ReceiveParam({0, 0}, {10, 10})};
  // Behavior second_receiver{
  //   Behavior::Type::OneTouchReceiveKick,
  //   Behavior::Priority::Required,
  //   ReceiveParam({10, 10}, {-10, -10})};
  // Behavior final_receiver_shot{
  //   Behavior::Type::OneTouchShot,
  //   Behavior::Priority::Required,
  //   ReceiveShotParam({-10, -10})};

  // std::size_t parent = three_one_touch_shot.add_node(initial_pass_start);
  // parent = three_one_touch_shot.add_node(first_receiver, parent);
  // parent = three_one_touch_shot.add_node(second_receiver, parent);
  // parent = three_one_touch_shot.add_node(final_receiver_shot, parent);


  // DirectedGraph<BehaviorGoal> direct_shot;
  // BehaviorGoal shot{
  //   Behavior::Type::Shot,
  //   Behavior::Priority::Required,
  //   ShotParam()};

  // direct_shot.add_node(shot);

  // Generate 16 move behaviors towards the ball, we'll figure out which
  // ones we can fill later
  // DirectedGraph<BehaviorGoal> simple_move;
  // static double j = 0;
  // j += kRotationSpeed;
  // for (int i = 0; i < 16; i++) {
  //   double theta = i / 11.0 * 3.14 * 2 + j;
  //   BehaviorGoal move{
  //     BehaviorGoal::Type::MoveToPoint,
  //     BehaviorGoal::Priority::Required,
  //     MoveParam(
  //       (sin(3 * theta) + 1.5) * Eigen::Vector2d{cos(theta), sin(
  //           theta)} - Eigen::Vector2d{2, 0})};
  // }

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
  }

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
  DirectedGraph<BehaviorGoal> behavior_out = qual_goalie_and_shot;

  return behavior_out;
}
