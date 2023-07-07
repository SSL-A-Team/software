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
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, overlay_publisher);
}

void WallPlay::reset(){
  for(auto & move_to : easy_move_tos_) {
    move_to.reset();
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

    ateam_geometry::Segment wall_line = ateam_geometry::Segment(
        ateam_geometry::Point(-3, 0.2 * current_available_robots.size() / 2.0),
        ateam_geometry::Point(-3, -0.2 * current_available_robots.size() / 2.0)
    );

    std::vector<ateam_geometry::Point> positions_to_assign =
        get_equally_spaced_points_on_segment(wall_line, current_available_robots.size());

    const auto & robot_assignments = robot_assignment::assign(current_available_robots, positions_to_assign);
    for (const auto [robot_id, pos_ind] : robot_assignments) {
      const auto & maybe_assigned_robot = world.our_robots.at(robot_id);

      if(!maybe_assigned_robot) {
        // TODO log this?
        continue;
      }

      const auto & robot = maybe_assigned_robot.value();

      auto & easy_move_to = easy_move_tos_.at(robot_id);

      easy_move_to.setTargetPosition(positions_to_assign.at(pos_ind));
      easy_move_to.setFacingTowards(world.ball.pos);
      maybe_motion_commands.at(robot_id) = easy_move_to.runFrame(robot, world);
    }

    int our_goalie_id = world.referee_info.our_goalie_id;
    if (auto maybe_goalie = world.our_robots.at(our_goalie_id)) {
      Robot our_goalie = maybe_goalie.value();
      auto goalie_point = ateam_kenobi::skills::get_goalie_defense_point(world);
      auto & goalie_move_to = easy_move_tos_.at(our_goalie_id);
      goalie_move_to.setTargetPosition(goalie_point);
      goalie_move_to.setFacingTowards(world.ball.pos);
      path_planning::PlannerOptions options;
      options.avoid_ball = false;
      goalie_move_to.setPlannerOptions(options);
      maybe_motion_commands.at(our_goalie_id) = goalie_move_to.runFrame(our_goalie, world);
    }

    return maybe_motion_commands;
};


} // namespace ateam_kenobi::plays