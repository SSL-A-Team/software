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


#include "our_kickoff_play.hpp"
#include "types/world.hpp"
#include "skills/goalie.hpp"
#include "skills/kick.hpp"
#include "robot_assignment.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{
OurKickoffPlay::OurKickoffPlay(visualization::OverlayPublisher & overlay_publisher, visualization::PlayInfoPublisher & play_info_publisher)
: BasePlay(overlay_publisher, play_info_publisher),
  goalie_skill_(overlay_publisher, play_info_publisher)
{
}

void OurKickoffPlay::reset()
{
  positions_to_assign_.clear();

  // Get a kicker
  ateam_geometry::Point kicker_point = ateam_geometry::Point(-0.55, 0);
  positions_to_assign_.push_back(kicker_point);
  // Get 4 defenders
  positions_to_assign_.push_back(ateam_geometry::Point(-0.3, -1.5));
  positions_to_assign_.push_back(ateam_geometry::Point(-0.3, 1.5));
  positions_to_assign_.push_back(ateam_geometry::Point(-2, 2));
  positions_to_assign_.push_back(ateam_geometry::Point(-2, -2));

  goalie_skill_.reset();
  attempted_to_kick_.fill(false);
  ready_to_kickoff_ = false;
}

void OurKickoffPlay::set_kickoff_ready(){
  ready_to_kickoff_ = true;
};

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> OurKickoffPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;
  std::vector<Robot> current_available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(current_available_robots, world);

  // TARGET POSITIONS
  // get closest robot to ball and its distance to the ball
  int kicker_id;
  double min_dist_to_ball = 1999;
  // If we haven't chosen a kicker yet, pick the closest robot to the ball
  if (prev_assigned_id_ == -1){
    for (auto robot : current_available_robots) {
      double cur_dist_to_ball = CGAL::squared_distance(world.ball.pos, robot.pos);
      if (cur_dist_to_ball < min_dist_to_ball) {
        if (!attempted_to_kick_.at(robot.id)){
          min_dist_to_ball = cur_dist_to_ball;
          kicker_id = robot.id;
        } 
      }
    }
  } else {
    kicker_id = prev_assigned_id_;
  }
  Robot kicker_bot;
  // Otherwise use our chosen kicker
  if (world.our_robots.at(kicker_id).has_value()){
    kicker_bot = world.our_robots.at(kicker_id).value();
  }
  else {
    // If we can't find the kicker for some reason, exit
    // and choose a different robot next time
    prev_assigned_id_ = -1;
    return maybe_motion_commands;
  }
  min_dist_to_ball = CGAL::squared_distance(world.ball.pos, kicker_bot.pos);
  // Get a kicker position
  if (min_dist_to_ball < -1.5) {
    if (min_dist_to_ball > -1.1) {
      positions_to_assign_.at(kicker_id) = ateam_geometry::Point(
        world.ball.pos.x() + -0.55, world.ball.pos.y());
    } else {
        // kick the ball and return
        auto their_goal = ateam_geometry::Point(-world.field.field_length / 2 + 0.2, 0);
        ready_to_kickoff_ = true;
        if (ready_to_kickoff_ && world.referee_info.running_command == ateam_common::GameCommand::NormalStart){
          maybe_motion_commands.at(kicker_id) = ateam_kenobi::skills::line_kick_command(world, kicker_bot,
            their_goal, easy_move_tos_.at(kicker_id));
          if (min_dist_to_ball < 0.0005){
            attempted_to_kick_.at(kicker_id) = true;
          }
        }
        if (world.our_robots.at(world.referee_info.our_goalie_id).has_value()){
        this->play_info_publisher_.message["Our Kickoff Play"]["robots"][kicker_id]["pos"] = {kicker_bot.pos.x(), kicker_bot.pos.y()};
  }
    }
  }
  // Assign remaining positions
  play_helpers::removeRobotWithId(current_available_robots, kicker_id);
  const auto & robot_assignments = robot_assignment::assign(current_available_robots, positions_to_assign_);
    for (const auto [robot_id, pos_ind] : robot_assignments) {
      const auto & maybe_assigned_robot = world.our_robots.at(robot_id);

      if(!maybe_assigned_robot) {
        // TODO log this?
        continue;
      }

      const Robot & robot = maybe_assigned_robot.value();
      this->play_info_publisher_.message["Our Kickoff Play"]["robots"][robot_id]["pos"] = {robot.pos.x(), robot.pos.y()};

      auto & easy_move_to = easy_move_tos_.at(robot_id);

      easy_move_to.setTargetPosition(positions_to_assign_.at(pos_ind));
      easy_move_to.setAngleMode(MotionOptions::AngleMode::face_point, world.ball.pos);
      maybe_motion_commands.at(robot_id) = easy_move_to.runFrame(robot, world);

    }
  // Have the goalie do stuff
  goalie_skill_.runFrame(world, maybe_motion_commands);
  if (world.our_robots.at(world.referee_info.our_goalie_id).has_value()){
      const Robot & goalie_robot = world.our_robots.at(world.referee_info.our_goalie_id).value();
      this->play_info_publisher_.message["Our Kickoff Play"]["robots"][world.referee_info.our_goalie_id]["pos"] = {goalie_robot.pos.x(), goalie_robot.pos.y()};
  }

  // Set our kicker to the robot we've chosen
  prev_assigned_id_ = kicker_id;

  play_info_publisher_.send_play_message("our_kickoff_play");

  return maybe_motion_commands;
}
}  // namespace ateam_kenobi::plays
