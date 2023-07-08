#include "our_penalty_play.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

OurPenaltyPlay::OurPenaltyPlay(
  visualization::OverlayPublisher & op,
  visualization::PlayInfoPublisher & pip)
  : BasePlay(op, pip),
    line_kick_skill_(op)
{
  play_helpers::EasyMoveTo::CreateArray(move_tos_, op);
}

void OurPenaltyPlay::reset()
{
  for(auto & move_to : move_tos_) {
    move_to.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> OurPenaltyPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  if(available_robots.empty()) {
    return {};
  }

  auto kicking_robot = available_robots.front();
  available_robots.erase(available_robots.begin());

  line_kick_skill_.setTargetPoint(ateam_geometry::Point(world.field.field_length/2.0, 0.0));

  if(world.in_play) {
    // Kick ball
    line_kick_skill_.runFrame(world, kicking_robot);
  } else {
    // Stage for kick
    const auto destination = line_kick_skill_.getAssignmentPoint(world);
    auto & move_to = move_tos_[kicking_robot.id];
    move_to.setTargetPosition(destination);
    move_to.face_travel();
    move_to.setMaxVelocity(1.5);
    motion_commands[kicking_robot.id] = move_to.runFrame(kicking_robot, world);
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

  return motion_commands;
}
}
