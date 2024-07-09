#include "their_ball_placement_play.hpp"
#include <angles/angles.h>
#include <ateam_common/robot_constants.hpp>
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

TheirBallPlacementPlay::TheirBallPlacementPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  easy_move_tos_(createIndexedChildren<play_helpers::EasyMoveTo>("EasyMoveTo"))
{}

stp::PlayScore TheirBallPlacementPlay::getScore(const World & world)
{
  const auto & cmd = world.referee_info.running_command;
  if (cmd == ateam_common::GameCommand::BallPlacementTheirs)
  {
    return stp::PlayScore::Max();
  }
  return stp::PlayScore::NaN();
}

void TheirBallPlacementPlay::reset()
{
  for (auto & emt : easy_move_tos_) {
    emt.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> TheirBallPlacementPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);

  ateam_geometry::Point placement_point = world.referee_info.designated_position;

  const auto point_to_ball = world.ball.pos - placement_point;
  const auto angle = std::atan2(point_to_ball.y(), point_to_ball.x());

  ateam_geometry::Polygon polygon_points;
  polygon_points.push_back(placement_point
    + 0.5 * ateam_geometry::Vector(std::cos(angle + M_PI/2), std::sin(angle + M_PI/2)));
  polygon_points.push_back(placement_point
    + 0.5 * ateam_geometry::Vector(std::cos(angle - M_PI/2), std::sin(angle - M_PI/2)));
  polygon_points.push_back(world.ball.pos
    + 0.5 * ateam_geometry::Vector(std::cos(angle - M_PI/2), std::sin(angle - M_PI/2)));
  polygon_points.push_back(world.ball.pos
    + 0.5 * ateam_geometry::Vector(std::cos(angle + M_PI/2), std::sin(angle + M_PI/2)));

  getOverlays().drawCircle("placement_avoid_point", ateam_geometry::makeCircle(placement_point, 0.5), "red", "red");
  getOverlays().drawCircle("placement_avoid_ball", ateam_geometry::makeCircle(world.ball.pos, 0.5), "red", "red");
  getOverlays().drawPolygon("placement_avoid_zone", polygon_points, "red", "red");

  // TODO: Get the other robots actually out of the way. Maybe just line up on the opposite side of the field
  for (auto ind = 0ul; ind < available_robots.size(); ++ind) {
    const auto & robot = available_robots[ind];
    auto & emt = easy_move_tos_[robot.id];

    const auto placement_segment = ateam_geometry::Segment(placement_point, world.ball.pos);
    const auto nearest_point = ateam_geometry::nearestPointOnSegment(placement_segment, robot.pos);

    ateam_geometry::Point target_position = robot.pos;
    if (ateam_geometry::norm(robot.pos - nearest_point) < 0.6 + kRobotRadius) {
      target_position = nearest_point
        + 0.7 * ateam_geometry::Vector(std::cos(angle + M_PI/2), std::sin(angle + M_PI/2));

      const auto alternate_position = nearest_point
        + 0.7 * ateam_geometry::Vector(std::cos(angle - M_PI/2), std::sin(angle - M_PI/2));

      
      if (ateam_geometry::norm(target_position - robot.pos) > ateam_geometry::norm(alternate_position - robot.pos)) {
        target_position = alternate_position;
      }

      getPlayInfo()["Robots"][std::to_string(robot.id)] = "MOVING";
    } else {
      getPlayInfo()["Robots"][std::to_string(robot.id)] = "-";
    }

    emt.setTargetPosition(target_position);
    emt.face_point(world.ball.pos);
    maybe_motion_commands[robot.id] = emt.runFrame(robot, world);
    
  }

  return maybe_motion_commands;
}

}  // namespace ateam_kenobi::plays