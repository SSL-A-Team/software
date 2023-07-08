#ifndef PLAY_HELPERS__AVAILABLE_ROBOTS_HPP_
#define PLAY_HELPERS__AVAILABLE_ROBOTS_HPP_

#include <vector>
#include "types/robot.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::play_helpers
{

std::vector<Robot> getAvailableRobots(const World & world);

std::vector<Robot> getVisibleRobots(const std::array<std::optional<Robot>, 16> &robots);

void removeGoalie(std::vector<Robot> & robots, const World & world);

void removeGoalie(std::array<std::optional<Robot>,16> & robots, const World & world);

void removeRobotWithId(std::vector<Robot> & robots, int id);

void removeRobotWithId(std::array<std::optional<Robot>,16> & robots, int id);
}

#endif  // PLAY_HELPERS__AVAILABLE_ROBOTS_HPP_
