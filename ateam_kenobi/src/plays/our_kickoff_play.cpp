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

namespace ateam_kenobi::plays
{
OurKickoffPlay::OurKickoffPlay(visualization::OverlayPublisher & overlay_publisher)
: BasePlay(overlay_publisher)
{
}

void OurKickoffPlay::reset()
{
  for (auto & path : saved_paths_) {
    if (path.has_value()) {
      path.value().clear();
    }
  }
  available_robots_.clear();
  for (auto & controller : motion_controllers_) {
    controller.reset();
  }
  positions_to_assign_.clear();

  // Get a kicker
  ateam_geometry::Point kicker_point = ateam_geometry::Point(-0.55, 0);
  positions_to_assign_.push_back(kicker_point);
  // Get 4 defenders
  positions_to_assign_.push_back(ateam_geometry::Point(-0.3, -1.5));
  positions_to_assign_.push_back(ateam_geometry::Point(-0.3, 1.5));
  positions_to_assign_.push_back(ateam_geometry::Point(-2, 2));
  positions_to_assign_.push_back(ateam_geometry::Point(-2, -2));
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> OurKickoffPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;
  std::vector<Robot> current_available_robots;

  // Get the number of available robots
  for (const auto & maybe_robot : world.our_robots) {
      if (maybe_robot && maybe_robot.value().id != world.referee_info.our_goalie_id) {
          current_available_robots.push_back(maybe_robot.value());
      }
  }

  if (current_available_robots.empty()) {
    return maybe_motion_commands;
  }

  available_robots_ = current_available_robots; // this global assignment seems dangerous in light of all the skips


  // TARGET POSITIONS
  // get closest robot to ball and its distance to the ball
  double min_dist_to_ball = 1999;
  int kicker_id;
  for (auto robot : available_robots_) {
    double cur_dist_to_ball = CGAL::squared_distance(world.ball.pos, robot.pos);
    if (cur_dist_to_ball < min_dist_to_ball) {
      min_dist_to_ball = cur_dist_to_ball;
      kicker_id = robot.id;
    }
  }
  if (min_dist_to_ball < -1.5) {
    if (min_dist_to_ball > -1.1) {
      positions_to_assign_.at(-1) = ateam_geometry::Point(
        world.ball.pos.x() + -1.05, world.ball.pos.y());
    } else {
      // kick the ball and return
      maybe_motion_commands.at(kicker_id) = ateam_kenobi::skills::send_kick_command();
      return maybe_motion_commands;
    }
  }

  // Generate new trajectories for all robots
  const auto & robot_assignments = robot_assignment::assign(available_robots_, positions_to_assign_);
  for (const auto [robot_id, pos_ind] : robot_assignments) {
    const auto & maybe_assigned_robot = world.our_robots.at(robot_id);

    // Yeah I dont care double check every single time no .values() without a has check
    if (maybe_assigned_robot.has_value()) {
      const auto & robot = maybe_assigned_robot.value();
      const auto & destination = positions_to_assign_.at(pos_ind);
      const auto path = path_planner_.getPath(robot.pos, destination, world, {});
      if (path.empty()) {
        overlay_publisher_.drawCircle(
          "highlight_invalid_robot",
          ateam_geometry::makeCircle(robot.pos, 0.2), "red", "transparent");
        return {};
      }
      saved_paths_.at(robot_id) = path;
    } else {
      // TODO Log this
      // Assigned non-available robot
      // ERROR  So this could happen because of the global assignment set
      // THIS WILL FAIL IF WE CONTINUE
      // continue;
    }

  }

  int our_goalie_id = world.referee_info.our_goalie_id;
  // Always get a new point and trajectory for the goalie so we can
  // be good at defense
  if (auto maybe_goalie = world.our_robots.at(our_goalie_id)) {
    Robot our_goalie = maybe_goalie.value();
    auto goalie_point = ateam_kenobi::skills::get_goalie_defense_point(world);
    const auto goalie_path = path_planner_.getPath(
        our_goalie.pos, goalie_point, world, {});
    motion_controllers_.at(our_goalie_id).set_trajectory(goalie_path);
    const auto current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
      world.current_time.time_since_epoch()).count();
    maybe_motion_commands.at(our_goalie_id) =  motion_controllers_.at(our_goalie_id).get_command(our_goalie, current_time);
  }

  // Execute trajectories, new or existing :)
  for (Robot robot : available_robots_){
    if (auto maybe_path = saved_paths_.at(robot.id)) {
      motion_controllers_.at(robot.id).set_trajectory(saved_paths_.at(robot.id).value());
      const auto current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
          world.current_time.time_since_epoch()).count();
      maybe_motion_commands.at(robot.id) = motion_controllers_.at(robot.id).get_command(robot, current_time);
    }
  }
  return maybe_motion_commands;
}
}  // namespace ateam_kenobi::plays
