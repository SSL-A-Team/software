#ifndef UTIL__IN_PLAY_EVAL_HPP_
#define UTIL__IN_PLAY_EVAL_HPP_

#include <Eigen/Dense>

#include <chrono>
#include <optional>

#include "types/world.hpp"
#include <ateam_common/game_controller_listener.hpp>


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
  ateam_common::GameCommand cur_state {ateam_common::GameCommand::Stop};
  std::optional<Eigen::Vector2d> maybe_kickoff_position {};   // optional in case ball cant be seen

  static constexpr double DIST_THRESHOLD {0.05};

  // only 3 things enter us into play according to the appendix B state machine
  void update(World world)
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
          in_play = false;
          break;

        // force play really play acts the same as play
        case ateam_common::GameCommand::ForceStart:
          in_play = true;
          break;
        default:
          in_play = true;
      }
    }

    cur_state = next_command;

    // level
    if (in_play == false) {
      auto maybe_ball_current = world.get_unique_ball();
      if (maybe_ball_current.has_value()) {
        if (maybe_kickoff_position.has_value()) {
          // if we are tracking a start position and a current ball and its further away than
          // dist we are in play
          if ((maybe_ball_current.value().pos - maybe_kickoff_position.value()).norm() >
            DIST_THRESHOLD)
          {
            in_play = true;
            // so if you read this and its any of the states in the above you are basically
            // playing under restrictions
          }
        } else {
          // set the ball position if we dont have one yet during this kickoff
          maybe_kickoff_position = maybe_ball_current.value().pos;
        }
      }
    } else {
      // note we may not have a view of the ball the frame we enter kickoff thats
      // why this is seperate
      maybe_kickoff_position = std::nullopt;
    }
  }
};

#endif  // UTIL__IN_PLAY_EVAL_HPP_
