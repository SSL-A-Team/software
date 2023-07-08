#include "available_robots.hpp"

namespace ateam_kenobi::play_helpers
{

std::vector<Robot> getAvailableRobots(const World &world)
{
  return getVisibleRobots(world.our_robots);
}

std::vector<Robot> getVisibleRobots(const std::array<std::optional<Robot>, 16> &robots)
{
  std::vector<Robot> available_robots;
  for (const auto & maybe_robot : robots) {
    if (maybe_robot) {
      available_robots.push_back(maybe_robot.value());
    }
  }
  return available_robots;
}

void removeGoalie(std::vector<Robot> &robots, const World & world)
{
  return removeRobotWithId(robots, world.referee_info.our_goalie_id);
}

void removeGoalie(std::array<std::optional<Robot>,16> &robots, const World & world)
{
  return removeRobotWithId(robots, world.referee_info.our_goalie_id);
}

void removeRobotWithId(std::vector<Robot> &robots, int id){
  robots.erase(std::remove_if(robots.begin(), robots.end(), [&id](const Robot & robot){
    return robot.id == id;
  }), robots.end());
}

void removeRobotWithId(std::array<std::optional<Robot>,16> &robots, int id)
{
  robots.at(id).reset();
}
}  // namespace ateam_kenobi::play_helpers
