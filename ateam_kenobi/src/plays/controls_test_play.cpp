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

#include "controls_test_play.hpp"
#include "ateam_geometry/types.hpp"
#include "ateam_geometry/make_circle.hpp"
#include "types/world.hpp"
#include "robot_assignment.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{
ControlsTestPlay::ControlsTestPlay(
  visualization::OverlayPublisher & overlay_publisher,
  visualization::PlayInfoPublisher & play_info_publisher)
: BasePlay(overlay_publisher, play_info_publisher)
{
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, overlay_publisher);

  this->points.push_back(ateam_geometry::Point(-0.5, -0.5));
  this->points.push_back(ateam_geometry::Point(0.5, -0.5));
  this->points.push_back(ateam_geometry::Point(0.5, 0.5));
  this->points.push_back(ateam_geometry::Point(-0.5, 0.5));

  motion_controller_.face_travel();
  motion_controller_.v_max = 1;
  motion_controller_.t_max = 3;
}

void ControlsTestPlay::reset()
{
  motion_controller_.reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> ControlsTestPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;
  auto current_available_robots = play_helpers::getAvailableRobots(world);

  if (current_available_robots.size() > 0) {
    const auto & robot = current_available_robots[0];
    int robot_id = robot.id;
   

    if (ateam_geometry::norm(robot.pos - this->points[this->index]) < 0.05 && ateam_geometry::norm(robot.vel) < 0.2) {
      this->index++;
      if (this->index >= this->points.size()) this->index = 0;
    }

    const std::vector<ateam_geometry::Point> path = {
      robot.pos,
      this->points[this->index]
    };
    overlay_publisher_.drawLine("controls_test_path", path, "purple");

    this->play_info_publisher_.message["robot"]["id"] = robot_id;
    this->play_info_publisher_.message["robot"]["target"]["x"] = this->points[this->index].x();
    this->play_info_publisher_.message["robot"]["target"]["y"] = this->points[this->index].y();
    this->play_info_publisher_.message["robot"]["pos"]["x"] = robot.pos.x();
    this->play_info_publisher_.message["robot"]["pos"]["y"] = robot.pos.y();
    this->play_info_publisher_.message["robot"]["vel"]["x"] = robot.vel.x();
    this->play_info_publisher_.message["robot"]["vel"]["y"] = robot.vel.y();


    motion_controller_.set_trajectory(std::vector<ateam_geometry::Point> {this->points[this->index]});

    const auto current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
    world.current_time.time_since_epoch()).count();
    maybe_motion_commands[robot_id] = motion_controller_.get_command(robot, current_time);

  }

  for (int i = 0; i < this->points.size(); i++) {
    overlay_publisher_.drawCircle("controls_test_point" + std::to_string(i), 
		                  ateam_geometry::makeCircle(this->points[i], .05), 
				  "blue",
				  "blue"
				  );
  }
  play_info_publisher_.send_play_message("Controls Test Play");

  return maybe_motion_commands;
}
}  // namespace ateam_kenobi::plays
