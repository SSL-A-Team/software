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

#include "stop_play.hpp"
#include <ateam_msgs/msg/robot_motion_command.hpp>

namespace ateam_kenobi::plays
{
StopPlay::StopPlay(visualization::OverlayPublisher & overlay_publisher)
: BasePlay(overlay_publisher)
{
}

void StopPlay::reset()
{
  for (int i = 0; i < 16; i++) {
    motion_controllers_[i].reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> StopPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> stop_motion_commands;
  for (size_t i = 0; i < 16; ++i){
    // only going to do 8 robots for now to avoid crowding on one side of the ball
    if (i < 8) {
      // avoid the ball by 0.7m just to be safe
      double radius = 0.7;
      const auto & maybe_robot = world.our_robots[i];
      if (maybe_robot.has_value()) {
        double x = (radius * cos(2*i*M_PI/8.0)) + world.ball.pos.x();
        double y = (radius * sin(2*i*M_PI/8.0)) + world.ball.pos.y();
        const auto & destination = ateam_geometry::Point(x, y);

        overlay_publisher_.drawCircle(
          "stop"+ std::to_string(i),
          ateam_geometry::makeCircle(destination, 0.15), "blue", "transparent");

        const auto & robot = maybe_robot.value();
        const auto path = path_planner_.getPath(robot.pos, destination, world, {});

        std::cerr << "robot " << i << "path size: " << path.size() << std::endl;

        auto & motion_controller = this->motion_controllers_[i];
        motion_controller.set_trajectory(path);
        motion_controller.face_towards = world.ball.pos; // face the ball

        const auto current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
          world.current_time.time_since_epoch()).count();

        const auto & motion_command = motion_controller.get_command(robot, current_time);
        std::cerr << "motion command has value: " << motion_command.twist.angular.z << std::endl;
        stop_motion_commands[i] = motion_command;
        continue;
        }
      }
      stop_motion_commands[i] = ateam_msgs::msg::RobotMotionCommand{};
  }
    // Draw Keepout Circle
    overlay_publisher_.drawCircle(
      "keepout_circle",
      ateam_geometry::makeCircle(world.ball.pos, 0.35), "red", "transparent");

    return stop_motion_commands;
}
}  // namespace ateam_kenobi::plays