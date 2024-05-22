// Copyright 2024 A Team
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


/* This executable is for finding path planner bugs by running random scenarios until a switchback
 * or loop is found. It's not run as part of our normal test suite and is intended to be run
 * manually when you're trying to find hard to reproduce path planner bugs
 */

#include <cstdio>
#include <cmath>
#include <random>
#include <ateam_geometry/normalize.hpp>
#include "path_planning/path_planner.hpp"

using ateam_kenobi::path_planning::PathPlanner;
using ateam_kenobi::path_planning::PlannerOptions;

bool isSwitchBack(
  const ateam_geometry::Point & a, const ateam_geometry::Point & b,
  const ateam_geometry::Point & c)
{
  const auto vec_ab = a - b;
  const auto vec_cb = c - b;
  const auto angle =
    std::acos((vec_ab * vec_cb) / (ateam_geometry::norm(vec_ab) * ateam_geometry::norm(vec_cb)));
  return angle < 0.0873;  // 5 degrees
}

bool pathContainsSwitchback(const PathPlanner::Path & path)
{
  if (path.empty()) {
    return false;
  }
  for (auto i = 1ul; i < path.size() - 1; ++i) {
    const auto & a = path[i - 1];
    const auto & b = path[i];
    const auto & c = path[i + 1];
    if (isSwitchBack(a, b, c)) {
      std::cout << "Found switchback.\n";
      return true;
    }
  }
  return false;
}

bool pathContainsLoops(const PathPlanner::Path & path)
{
  if (path.size() < 4) {
    return false;
  }
  for (auto ind1 = 0ul; ind1 < path.size() - 3; ++ind1) {
    for (auto ind2 = ind1 + 2; ind2 < path.size() - 1; ++ind2) {
      const ateam_geometry::Segment seg1(path[ind1], path[ind1 + 1]);
      const ateam_geometry::Segment seg2(path[ind2], path[ind2 + 1]);
      if (CGAL::do_intersect(seg1, seg2)) {
        std::cout << "Found loop\n";
        return true;
      }
    }
  }
  return false;
}

bool isBadPath(const PathPlanner::Path & path)
{
  /* Note: Empty or incomplete paths are not considered bad since there is no guarantee that our
   * random scenarios are solvable.
   */
  return pathContainsSwitchback(path) ||
         pathContainsLoops(path);
}

void printUsage()
{
  std::cout << "Usage: random_path_planner_test [-t timeout] [-r]\n";
  std::cout << "-t timeout:\n";
  std::cout << "\tdefault = 10\n";
  std::cout << "\tHow long, in minutes, to run the test if no failures are found.\n";
  std::cout << "-r:\n";
  std::cout << "\tRender the failing scenario via python + matplotlib.\n";
}

void renderSecenario(
  const ateam_geometry::Point & start, const ateam_geometry::Point & goal,
  const ateam_geometry::Point & ball_pos,
  const ateam_geometry::Point & opponent_pos, const PathPlanner::Path & path)
{
  std::stringstream script;
  script << "import matplotlib.pyplot as plt\n";
  script << "import numpy as np\n";
  script << "ball_radius = 0.04267 / 2.0\n";
  script << "robot_radius = 0.09\n";
  script << "start = (" << start.x() << ", " << start.y() << ")\n";
  script << "goal = (" << goal.x() << ", " << goal.y() << ")\n";
  script << "ball = (" << ball_pos.x() << ", " << ball_pos.y() << ")\n";
  script << "opponent = (" << opponent_pos.x() << ", " << opponent_pos.y() << ")\n";
  script << "path = np.array([\n";
  for (const auto & p : path) {
    script << "[" << p.x() << ", " << p.y() << "],\n";
  }
  script << "])\n";
  script << "_, axes = plt.subplots()\n";
  script << "plt.plot(path[:, 0], path[:, 1], '-o')\n";
  script << "axes.add_artist(plt.Circle(start, robot_radius, color='blue'))\n";
  script << "axes.add_artist(plt.Circle(goal, robot_radius, color='green'))\n";
  script << "axes.add_artist(plt.Circle(opponent, robot_radius, color='red'))\n";
  script << "axes.add_artist(plt.Circle(ball, ball_radius, color='orange'))\n";
  script << "plt.show()\n";

  FILE * pystdin = popen("python3 -", "w");
  if (pystdin == nullptr) {
    std::cerr << "Failed to start python interpretter.\n";
    return;
  }

  const auto script_string = script.str();
  const auto num_bytes_written = fwrite(script_string.c_str(), 1, script_string.size(), pystdin);
  if (num_bytes_written < script_string.size()) {
    std::cerr << "Failed to send script to python interpretter.\n";
  }

  const auto exit_code = pclose(pystdin);
  if (exit_code != 0) {
    std::cerr << "Python interpretter ended with non-zero exit code: " << exit_code << '\n';
  }
}

int main(int argc, char ** argv)
{
  std::chrono::minutes timeout(10);
  bool render_scenario = false;

  for (int argind = 0; argind < argc; ++argind) {
    const std::string arg = argv[argind];
    if (arg == "-t") {
      if (argc - argind < 1) {
        printUsage();
        return 1;
      }
      timeout = std::chrono::minutes(std::stoi(argv[++argind]));
    }
    if (arg == "-r") {
      render_scenario = true;
    }
  }

  std::random_device r;
  std::default_random_engine e1(r());
  std::uniform_real_distribution<double> uniform_dist(0.0, 0.3);
  auto randNum = [&uniform_dist, &e1]() {
      return uniform_dist(e1);
    };

  PlannerOptions planner_options;
  planner_options.ignore_start_obstacle = false;

  ateam_kenobi::World world;
  world.field.field_length = 9;
  world.field.field_width = 6;
  world.field.boundary_width = 0.01;
  // TODO(barulicm) change these fields when the field geometry fix lands
  world.field.goal_width = 2;
  world.field.goal_depth = 1;
  world.their_robots[0].visible = true;
  world.their_robots[0].id = 0;

  PathPlanner path_planner;

  const auto start_time = std::chrono::steady_clock::now();

  while (true) {
    const ateam_geometry::Point start(randNum(), randNum());
    const ateam_geometry::Point goal(randNum(), randNum());
    world.ball.pos = ateam_geometry::Point(randNum(), randNum());
    world.their_robots[0].pos = ateam_geometry::Point(randNum(), randNum());

    const auto path = path_planner.getPath(start, goal, world, {}, planner_options);

    if (isBadPath(path)) {
      std::cout << "Bad path found!\n";
      std::cout << "start = " << start << '\n';
      std::cout << "goal = " << goal << '\n';
      std::cout << "ball = " << world.ball.pos << '\n';
      std::cout << "opponent = " << world.their_robots[0].pos << '\n';

      std::cout << "path =";
      for (const auto & p : path) {
        std::cout << p << '\n';
      }

      if (render_scenario) {
        renderSecenario(start, goal, world.ball.pos, world.their_robots[0].pos, path);
      }

      break;
    }

    if ((std::chrono::steady_clock::now() - start_time) > timeout) {
      std::cout << "No bad paths found. Timed out.\n";
      break;
    }
  }

  return 0;
}
