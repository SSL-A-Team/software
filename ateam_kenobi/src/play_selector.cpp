#include "play_selector.hpp"

namespace ateam_kenobi
{

PlaySelector::PlaySelector(visualization::OverlayPublisher & overlay_publisher)
  : test_play_(overlay_publisher)
{

}

plays::BasePlay * PlaySelector::getPlay(const World & world)
{
  plays::BasePlay * selected_play;

  //
  // Play selection logic goes here
  //
  selected_play = &test_play_;

  // ateam_common::GameCommand running_command = world.referee_info.running_command;
  // ateam_common::GameCommand prev_command = world.referee_info.prev_command;

  // in_play_eval_.update(world);
  // switch (running_command) {
  //   case ateam_common::GameCommand::Halt:
  //     behavior_out = generate_halt(world);
  //     break;
  //   case ateam_common::GameCommand::Stop:
  //     behavior_out = generate_halt(world);
  //     break;
  //   case ateam_common::GameCommand::NormalStart:
  //     if (in_play_eval_.in_play) {
  //       // free play
  //       return open_play(world);
  //     } else {
  //       switch (prev_command) {
  //         case ateam_common::GameCommand::PrepareKickoffOurs:
  //           // we can kick now only with the kicker, movement lock still in effect
  //           // NOTE GENERATE BASIC SHOOT SENDS ONE ROBOT TO THE BALL TO DO THINGS
  //           behavior_out = generate_basic_shoot(world);
  //           break;
  //         case ateam_common::GameCommand::PrepareKickoffTheirs:
  //           // we have to wait for them to kick, movement lock still in effect
  //           behavior_out = generate_basic_defense(world);
  //           break;
  //         case ateam_common::GameCommand::DirectFreeOurs:
  //           // FREE KICK
  //           // we can kick now only with the kicker, movement lock still in effect
  //           // TODO(CAVIDANO) WHAT SHOULD I DO HERE
  //           behavior_out = generate_basic_defense(world);
  //           break;
  //         case ateam_common::GameCommand::DirectFreeTheirs:
  //           // FREE KICK
  //           // we have to wait for them to kick, movement lock still in effect
  //           behavior_out = generate_basic_defense(world);
  //           break;
  //         case ateam_common::GameCommand::PreparePenaltyOurs:
  //           // we can kick now only with the kicker, no double touch applies here AND WE CAN ONLY MOVE TORWARDS THE GOAL
  //           behavior_out = do_our_penalty_kick(world);
  //           break;
  //         case ateam_common::GameCommand::PreparePenaltyTheirs:
  //           // we have to wait for them to kick, movement lock still in effect
  //           behavior_out = do_their_penalty_kick(world);
  //           break;
  //         default:
  //           break;
  //       }

  //     }
  //     break;
  //   case ateam_common::GameCommand::ForceStart:
  //     behavior_out = generate_basic_shoot(world);
  //     break;
  //   case ateam_common::GameCommand::PrepareKickoffOurs:
  //     behavior_out = setup_our_kickoff(world);
  //     break;
  //   case ateam_common::GameCommand::PrepareKickoffTheirs:
  //     break;
  //   case ateam_common::GameCommand::PreparePenaltyOurs:
  //     behavior_out = prepare_our_penalty_kick(world);
  //     break;
  //   case ateam_common::GameCommand::PreparePenaltyTheirs:
  //     behavior_out = prepare_their_penalty_kick(world);
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
  //     behavior_out = generate_halt(world);
  //     break;

  //   case ateam_common::GameCommand::BallPlacementOurs:
  //     break;
  //   case ateam_common::GameCommand::BallPlacementTheirs:
  //     break;

  // }

  // return behavior_out;

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
