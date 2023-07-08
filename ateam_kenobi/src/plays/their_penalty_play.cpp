#include "their_penalty_play.hpp"
#include "play_helpers/available_robots.hpp"
#include <ateam_common/robot_constants.hpp>

namespace ateam_kenobi::plays
{

TheirPenaltyPlay::TheirPenaltyPlay(
  visualization::OverlayPublisher & op,
  visualization::PlayInfoPublisher & pip)
  : BasePlay(op, pip),
    goalie_skill_(op, pip)
{
  play_helpers::EasyMoveTo::CreateArray(move_tos_, op);
}

void TheirPenaltyPlay::reset()
{
  for(auto & move_to : move_tos_) {
    move_to.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> TheirPenaltyPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  if(available_robots.empty()) {
    return {};
  }

  if(world.in_play) {
    goalie_skill_.runFrame(world, motion_commands);
  } else {
    const auto robot_id = world.referee_info.our_goalie_id;
    const auto & maybe_robot = world.our_robots[robot_id];
    if(maybe_robot) {
      const auto & robot = maybe_robot.value();
      auto & move_to = move_tos_[robot_id];
      move_to.setTargetPosition(ateam_geometry::Point(-world.field.field_length/2.0, 0.0));
      move_to.face_absolute(0.0);
      path_planning::PlannerOptions options;
      options.use_default_obstacles = false;
      move_to.setPlannerOptions(options);
      motion_commands[robot_id] = move_to.runFrame(robot, world);

    }
  }

  auto i = 0;
  ateam_geometry::Point pattern_start(kRobotDiameter-(world.field.field_length / 2.0), kRobotDiameter-(world.field.field_width/2.0));
  ateam_geometry::Vector pattern_step(kRobotDiameter + 0.2, 0.0);
  for(const auto & robot : available_robots) {
    auto & move_to = move_tos_[robot.id];
    move_to.setTargetPosition(pattern_start + (i * pattern_step));
    move_to.setMaxVelocity(1.5);
    move_to.face_travel();
    motion_commands[robot.id] = move_to.runFrame(robot, world);
    i++;
  }

  play_info_publisher_.send_play_message("Their Penalty Play");
  return motion_commands;
}
}
