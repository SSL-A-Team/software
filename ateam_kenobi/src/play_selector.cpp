#include <iostream>
#include "play_selector.hpp"
#include "ateam_common/game_controller_listener.hpp"

namespace ateam_kenobi
{

PlaySelector::PlaySelector(
  visualization::OverlayPublisher & overlay_publisher,
  visualization::PlayInfoPublisher & play_info_publisher)
: test_play_(overlay_publisher, play_info_publisher),
  halt_play_(overlay_publisher, play_info_publisher),
  stop_play_(overlay_publisher, play_info_publisher),
  wall_play_(overlay_publisher, play_info_publisher),
  our_kickoff_play_(overlay_publisher, play_info_publisher),
  test_kick_play_(overlay_publisher, play_info_publisher),
  basic_122_play_(overlay_publisher, play_info_publisher),
  our_penalty_play_(overlay_publisher, play_info_publisher),
  their_penalty_play_(overlay_publisher, play_info_publisher)
{
}

plays::BasePlay * PlaySelector::getPlay(const World & world)
{
  ateam_common::GameCommand current_game_command = world.referee_info.running_command;

  if (current_game_command == ateam_common::GameCommand::Halt) {
    return finalizeSelection(&halt_play_, current_game_command);
  }

  if (current_game_command == ateam_common::GameCommand::Stop) {
    return finalizeSelection(&stop_play_, current_game_command);
  }

  if (world.our_penalty) {
    return finalizeSelection(&our_penalty_play_, current_game_command);
  }

  if (world.their_penalty) {
    return finalizeSelection(&their_penalty_play_, current_game_command);
  }

  // if(world.in_play) {
  //   return finalizeSelection(&basic_122_play_, current_game_command);
  // }

  plays::BasePlay * selected_play = &halt_play_;

  switch (current_game_command) {
    case ateam_common::GameCommand::Halt:
    case ateam_common::GameCommand::TimeoutOurs:
    case ateam_common::GameCommand::TimeoutTheirs:
    case ateam_common::GameCommand::GoalOurs:
    case ateam_common::GameCommand::GoalTheirs:
      selected_play = &halt_play_;
      break;
    case ateam_common::GameCommand::Stop:
      selected_play = &stop_play_;
      break;
    case ateam_common::GameCommand::NormalStart:
      return finalizeSelection(pickNormalStartPlay(), current_game_command);
    case ateam_common::GameCommand::ForceStart:
      selected_play = &basic_122_play_;
      break;
    case ateam_common::GameCommand::PrepareKickoffOurs:
      selected_play = &our_kickoff_play_;
      break;
    case ateam_common::GameCommand::PrepareKickoffTheirs:
      selected_play = &wall_play_;
      break;
    case ateam_common::GameCommand::PreparePenaltyOurs:
      selected_play = &our_penalty_play_;
      break;
    case ateam_common::GameCommand::PreparePenaltyTheirs:
      selected_play = &their_penalty_play_;
      break;
    case ateam_common::GameCommand::DirectFreeOurs:
    case ateam_common::GameCommand::IndirectFreeOurs:
      selected_play = &basic_122_play_;
      break;
    case ateam_common::GameCommand::DirectFreeTheirs:
    case ateam_common::GameCommand::IndirectFreeTheirs:
      selected_play = &wall_play_;
      break;
    case ateam_common::GameCommand::BallPlacementOurs:
    case ateam_common::GameCommand::BallPlacementTheirs:
      selected_play = &stop_play_;
      break;
  }

  return finalizeSelection(selected_play, current_game_command);
}

void PlaySelector::resetPlayIfNeeded(plays::BasePlay * play)
{
  void * play_address = static_cast<void *>(play);
  if (play_address != prev_play_address_) {
    if (play != nullptr) {
      play->reset();
    }
    prev_play_address_ = play_address;
  }
}

plays::BasePlay * PlaySelector::finalizeSelection(
  plays::BasePlay * play,
  ateam_common::GameCommand current_game_command)
{
  resetPlayIfNeeded(play);

  if (current_game_command != previous_game_command_) {
    previous_game_command_ = current_game_command;
  }

  return play;
}

plays::BasePlay *PlaySelector::pickNormalStartPlay()
{
  switch(previous_game_command_) {
    case ateam_common::GameCommand::PrepareKickoffOurs:
      return &our_kickoff_play_;
    case ateam_common::GameCommand::PrepareKickoffTheirs:
      return &wall_play_;
    case ateam_common::GameCommand::IndirectFreeOurs:
    case ateam_common::GameCommand::DirectFreeOurs:
      return &basic_122_play_;
      break;
    case ateam_common::GameCommand::IndirectFreeTheirs:
    case ateam_common::GameCommand::DirectFreeTheirs:
      return &wall_play_;
      break;
    case ateam_common::GameCommand::PreparePenaltyOurs:
      return &our_penalty_play_;
      break;
    case ateam_common::GameCommand::PreparePenaltyTheirs:
      return &their_penalty_play_;
      break;
    default:
      return &basic_122_play_;
  }
  return &halt_play_;
}

} // namespace ateam_kenobi
