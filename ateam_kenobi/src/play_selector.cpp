#include "play_selector.hpp"

namespace ateam_kenobi
{

PlaySelector::PlaySelector(visualization::OverlayPublisher & overlay_publisher, visualization::PlayInfoPublisher & play_info_publisher)
  : test_play_(overlay_publisher, play_info_publisher), halt_play_(overlay_publisher, play_info_publisher), stop_play_(overlay_publisher, play_info_publisher), wall_play_(overlay_publisher, play_info_publisher)
{
}

plays::BasePlay * PlaySelector::getPlay(const World & world)
{
  plays::BasePlay * selected_play;

  ateam_common::GameCommand current_game_command = world.referee_info.running_command;
  //
  // Play selection logic goes here
  //
  if (current_game_command == ateam_common::GameCommand::Halt) {
    selected_play = &halt_play_;
  } else if (current_game_command == ateam_common::GameCommand::Stop) {
    selected_play = &stop_play_;
  } else {
    selected_play = &wall_play_;
  }

  // ateam_common::GameCommand running_command = world.referee_info.running_command;
  // ateam_common::GameCommand prev_command = world.referee_info.prev_command;

  // switch (running_command) {
  //   case ateam_common::GameCommand::Halt:
  //     selected_play = generate_halt(world);
  //     break;
  //   case ateam_common::GameCommand::Stop:
  //     selected_play = generate_halt(world);
  //     break;
  //   case ateam_common::GameCommand::NormalStart:
  //     if (world.in_play) {
  //       // free play
  //       return open_play(world);
  //     } else {
  //       switch (prev_command) {
  //         case ateam_common::GameCommand::PrepareKickoffOurs:
  //           // we can kick now only with the kicker, movement lock still in effect
  //           // NOTE GENERATE BASIC SHOOT SENDS ONE ROBOT TO THE BALL TO DO THINGS
  //           selected_play = generate_basic_shoot(world);
  //           break;
  //         case ateam_common::GameCommand::PrepareKickoffTheirs:
  //           // we have to wait for them to kick, movement lock still in effect
  //           selected_play = generate_basic_defense(world);
  //           break;
  //         case ateam_common::GameCommand::DirectFreeOurs:
  //           // FREE KICK
  //           // we can kick now only with the kicker, movement lock still in effect
  //           // TODO(CAVIDANO) WHAT SHOULD I DO HERE
  //           selected_play = generate_basic_defense(world);
  //           break;
  //         case ateam_common::GameCommand::DirectFreeTheirs:
  //           // FREE KICK
  //           // we have to wait for them to kick, movement lock still in effect
  //           selected_play = generate_basic_defense(world);
  //           break;
  //         case ateam_common::GameCommand::PreparePenaltyOurs:
  //           // we can kick now only with the kicker, no double touch applies here AND WE CAN ONLY MOVE TORWARDS THE GOAL
  //           selected_play = do_our_penalty_kick(world);
  //           break;
  //         case ateam_common::GameCommand::PreparePenaltyTheirs:
  //           // we have to wait for them to kick, movement lock still in effect
  //           selected_play = do_their_penalty_kick(world);
  //           break;
  //         default:
  //           break;
  //       }

  //     }
  //     break;
  //   case ateam_common::GameCommand::ForceStart:
  //     selected_play = generate_basic_shoot(world);
  //     break;
  //   case ateam_common::GameCommand::PrepareKickoffOurs:
  //     selected_play = setup_our_kickoff(world);
  //     break;
  //   case ateam_common::GameCommand::PrepareKickoffTheirs:
  //     break;
  //   case ateam_common::GameCommand::PreparePenaltyOurs:
  //     selected_play = prepare_our_penalty_kick(world);
  //     break;
  //   case ateam_common::GameCommand::PreparePenaltyTheirs:
  //     selected_play = prepare_their_penalty_kick(world);
  //     break;

  //   case ateam_common::GameCommand::DirectFreeOurs:
  //   case ateam_common::GameCommand::IndirectFreeOurs:

  //     break;

  //   case ateam_common::GameCommand::DirectFreeTheirs:
  //   case ateam_common::GameCommand::IndirectFreeTheirs:

  //     break;

  //   case ateam_common::GameCommand::GoalOurs:
  //   case ateam_common::GameCommand::GoalTheirs:
  //   case ateam_common::GameCommand::TimeoutOurs:
  //   case ateam_common::GameCommand::TimeoutTheirs:
  //     selected_play = generate_halt(world);
  //     break;

  //   case ateam_common::GameCommand::BallPlacementOurs:
  //     break;
  //   case ateam_common::GameCommand::BallPlacementTheirs:
  //     break;

  // }

  resetPlayIfNeeded(selected_play);

  return selected_play;
}

void PlaySelector::resetPlayIfNeeded(plays::BasePlay * play)
{
  void * play_address = static_cast<void*>(play);
  if(play_address != prev_play_address_) {
    if(play != nullptr) {
      play->reset();
    }
    prev_play_address_ = play_address;
  }
}

} // namespace ateam_kenobi
