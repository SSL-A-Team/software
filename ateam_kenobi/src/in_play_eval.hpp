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


#ifndef UTIL__IN_PLAY_EVAL_HPP_
#define UTIL__IN_PLAY_EVAL_HPP_

#include <math.h>
#include <Eigen/Dense>

#include <chrono>
#include <optional>

#include "types/world.hpp"
#include <ateam_geometry/types.hpp>
#include <ateam_common/game_controller_listener.hpp>

#include <CGAL/squared_distance_2.h>


using namespace std::chrono_literals;

/**
 *  Small class to be "ticked" with the world output_state and keep track of if we enter a kickoff and then when we exit (ball has moved 0.05 after normal_start is issued, 10 seconds pass, force start is issued)
 *
*/
class InPlayEval
{
public:
  // Ctors
  InPlayEval() {}

  // Vars
  bool in_play {};
  bool our_penalty {};
  bool their_penalty {};
  ateam_common::GameCommand cur_state {ateam_common::GameCommand::Stop};
  std::optional<ateam_geometry::Point> maybe_kickoff_position {};   // optional in case ball cant be seen

  static constexpr double DIST_THRESHOLD {0.05};

  // only 3 things enter us into play according to the appendix B state machine
  void update(ateam_kenobi::World & world)
  {
    // edge triggered
    // we just need an edge on commands to know not to do in_play = false in the same normal_play
    ateam_common::GameCommand next_command = world.referee_info.running_command;
    if (cur_state != next_command) {
      switch (next_command) {
        // all of these are not in play technically but not for this purpose
        case ateam_common::GameCommand::Halt:
        case ateam_common::GameCommand::Stop:
        case ateam_common::GameCommand::TimeoutTheirs:

        case ateam_common::GameCommand::PrepareKickoffOurs:
        case ateam_common::GameCommand::PrepareKickoffTheirs:
        case ateam_common::GameCommand::DirectFreeOurs:
        case ateam_common::GameCommand::DirectFreeTheirs:
        case ateam_common::GameCommand::BallPlacementOurs:
        case ateam_common::GameCommand::BallPlacementTheirs:
        case ateam_common::GameCommand::NormalStart:
          // reset in_play in any of these state
          world.in_play = false;
          break;

        case ateam_common::GameCommand::PreparePenaltyOurs:
          world.in_play = false;
          world.our_penalty = true;
          world.their_penalty = false;
          break;
        case ateam_common::GameCommand::PreparePenaltyTheirs:
          world.in_play = false;
          world.their_penalty = true;
          world.our_penalty = false;
          break;

        // force play really play acts the same as play
        case ateam_common::GameCommand::ForceStart:
          world.in_play = true;
          world.their_penalty = false;
          world.our_penalty = false;
          break;
        default:
          world.in_play = true;
          world.their_penalty = false;
          world.our_penalty = false;
      }
    }

    cur_state = next_command;

    // level
    if (in_play == false) {
      auto ball_current = world.ball;
      if (maybe_kickoff_position.has_value()) {
        // if we are tracking a start position and a current ball and its further away than
        // dist we are in play
        if (sqrt(CGAL::squared_distance(ball_current.pos, maybe_kickoff_position.value())) >
          DIST_THRESHOLD)
        {
          world.in_play = true;
          // so if you read this and its any of the states in the above you are basically
          // playing under restrictions
        }
      } else {
        // set the ball position if we dont have one yet during this kickoff
        maybe_kickoff_position = ball_current.pos;
      }
    } else {
      // note we may not have a view of the ball the frame we enter kickoff thats
      // why this is seperate
      maybe_kickoff_position = std::nullopt;
    }
    return;
  }
};

#endif  // UTIL__IN_PLAY_EVAL_HPP_
