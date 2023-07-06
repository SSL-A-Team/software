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

#include <CGAL/point_generators_2.h>

#include "wall_play.hpp"
#include "robot_assignment.hpp"
#include "types/robot.hpp"
#include "skills/goalie.hpp"

namespace ateam_kenobi::plays
{
    std::vector<ateam_geometry::Point> get_equally_spaced_points_on_segment(ateam_geometry::Segment & segment, int num_points){
        std::vector<ateam_geometry::Point> points_on_segment;

        if (num_points == 1){
            points_on_segment.push_back(segment.vertex(0));
        }

        auto source = segment.vertex(0);
        auto target = segment.vertex(1);

        ateam_geometry::Point spacing = ateam_geometry::Point(
            // source.x() + target.x() / (num_points - 1),
            0,
            CGAL::approximate_sqrt((source - target).squared_length()) / (num_points - 1)
        );

        for (int i = 0; i < num_points; ++i){
            auto segment_point = ateam_geometry::Point(
                // source.x() + (spacing.x() * i),
                target.x(),
                target.y() + (spacing.y() * i)
                // both of these were source in the last nothing else changed
            );

            points_on_segment.push_back(segment_point);
            // WHAT THE ACTUAL FUCK
        }
        return points_on_segment;
    }


WallPlay::WallPlay(visualization::OverlayPublisher & overlay_publisher)
: BasePlay(overlay_publisher)
{
}

void WallPlay::reset(){
    for (auto & path : saved_paths_) {
      if (path.has_value()) {
        path.value().clear();
      }
    }
    available_robots_.clear();
    for (auto & controller : motion_controllers_) {
      controller.reset();
    }
};

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> WallPlay::runFrame(
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
    // Used to create a line in front of the goal
    ateam_geometry::Segment wall_line = ateam_geometry::Segment(
        ateam_geometry::Point(-3, 0.2 * available_robots_.size() / 2.0),
        ateam_geometry::Point(-3, -0.2 * available_robots_.size() / 2.0)
    );

    std::vector<ateam_geometry::Point> positions_to_assign =
        get_equally_spaced_points_on_segment(wall_line, available_robots_.size());

    // Generate new trajectories for all robots
    const auto & robot_assignments = robot_assignment::assign(available_robots_, positions_to_assign);
    for (const auto [robot_id, pos_ind] : robot_assignments) {
      const auto & maybe_assigned_robot = world.our_robots.at(robot_id);

      // Yeah I dont care double check every single time no .values() without a has check
      if (maybe_assigned_robot.has_value()) {
        const auto & robot = maybe_assigned_robot.value();
        const auto & destination = positions_to_assign.at(pos_ind);
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

    // Always get a new point and trajectory for the goalie so we can
    // be good at defense
    int our_goalie_id = world.referee_info.our_goalie_id;
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
};


} // namespace ateam_kenobi::plays