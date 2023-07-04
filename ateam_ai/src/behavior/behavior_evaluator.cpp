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
#include "plays/halt.hpp"
#include "plays/kickoff.hpp"
#include "plays/shoot.hpp"
#include "plays/stop.hpp"
#include "plays/penalty_kick.hpp"

#include "plays/defense.hpp"
#include "plays/halt.hpp"
#include "plays/kickoff.hpp"
#include "plays/shoot.hpp"


CREATE_PARAM(double, "behavior_evaluator", kRotationSpeed, 0.005);

BehaviorEvaluator::BehaviorEvaluator(BehaviorRealization & behavior_realization)
: behavior_realization(behavior_realization) {}


// just for the 2 places we are then in open play
DirectedGraph<BehaviorGoal> BehaviorEvaluator::open_play(const World & world)
{
  DirectedGraph<BehaviorGoal>behavior_out =
    generate_basic_shoot(world).copy_from(generate_basic_defense(world));
  return behavior_out;
}

DirectedGraph<BehaviorGoal> BehaviorEvaluator::get_best_behaviors(const World & world)
{
  //
  // Do preprocessing on world state to get important metrics like possession
  //

  //
  // Setup different behavior options
  //

  // All common coding sense went out the window at about 9pm so.....
  // ateam_common::GameStage current_game_stage = world.referee_info.current_game_stage;
  ateam_common::GameCommand running_command = world.referee_info.running_command;
  ateam_common::GameCommand prev_command = world.referee_info.prev_command;

  in_play_eval.update(world);
  DirectedGraph<BehaviorGoal> behavior_out {};

  switch (running_command) {
    case ateam_common::GameCommand::Halt:
      behavior_out = generate_halt(world);
      break;
    case ateam_common::GameCommand::Stop:
      behavior_out = generate_halt(world);
      break;
    case ateam_common::GameCommand::NormalStart:
      if (in_play_eval.in_play) {
        // free play
        return open_play(world);
      } else {
        switch (prev_command) {
          case ateam_common::GameCommand::PrepareKickoffOurs:
            // we can kick now only with the kicker, movement lock still in effect
            // NOTE GENERATE BASIC SHOOT SENDS ONE ROBOT TO THE BALL TO DO THINGS
            behavior_out = generate_basic_shoot(world);
            break;
          case ateam_common::GameCommand::PrepareKickoffTheirs:
            // we have to wait for them to kick, movement lock still in effect
            behavior_out = generate_basic_defense(world);
            break;
          case ateam_common::GameCommand::DirectFreeOurs:
            // FREE KICK
            // we can kick now only with the kicker, movement lock still in effect
            // TODO(CAVIDANO) WHAT SHOULD I DO HERE
            behavior_out = generate_basic_defense(world);
            break;
          case ateam_common::GameCommand::DirectFreeTheirs:
            // FREE KICK
            // we have to wait for them to kick, movement lock still in effect
            behavior_out = generate_basic_defense(world);
            break;
          case ateam_common::GameCommand::PreparePenaltyOurs:
            // we can kick now only with the kicker, no double touch applies here AND WE CAN ONLY MOVE TORWARDS THE GOAL
            behavior_out = do_our_penalty_kick(world);
            break;
          case ateam_common::GameCommand::PreparePenaltyTheirs:
            // we have to wait for them to kick, movement lock still in effect
            behavior_out = do_their_penalty_kick(world);
            break;
          default:
            break;
        }

      }
      break;
    case ateam_common::GameCommand::ForceStart:
      behavior_out = generate_basic_shoot(world);
      break;
    case ateam_common::GameCommand::PrepareKickoffOurs:
      behavior_out = setup_our_kickoff(world);
      break;
    case ateam_common::GameCommand::PrepareKickoffTheirs:
      break;
    case ateam_common::GameCommand::PreparePenaltyOurs:
      behavior_out = prepare_our_penalty_kick(world);
      break;
    case ateam_common::GameCommand::PreparePenaltyTheirs:
      behavior_out = prepare_their_penalty_kick(world);
      break;

    case ateam_common::GameCommand::DirectFreeOurs:
    case ateam_common::GameCommand::IndirectFreeOurs:

      break;

    case ateam_common::GameCommand::DirectFreeTheirs:
    case ateam_common::GameCommand::IndirectFreeTheirs:

      break;

    case ateam_common::GameCommand::GoalOurs:
    case ateam_common::GameCommand::GoalTheirs:
    case ateam_common::GameCommand::TimeoutOurs:
    case ateam_common::GameCommand::TimeoutTheirs:
      behavior_out = generate_halt(world);
      break;

    case ateam_common::GameCommand::BallPlacementOurs:
      break;
    case ateam_common::GameCommand::BallPlacementTheirs:
      break;

  }

  return behavior_out;

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
  // DirectedGraph<BehaviorGoal> behavior_out = qual_goalie_and_shot;

  // return behavior_out;
}
