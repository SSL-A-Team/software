
#include "robot_assignment.hpp"

namespace ateam_kenobi::play_helpers
{

std::vector<std::optional<Robot>> assignRobots(
  const std::vector<Robot> & available_robots,
  const std::vector<ateam_geometry::Point> & positions,
  const std::vector<std::vector<int>> & disallowed_robot_ids)
{
  assert(disallowed_robot_ids.empty() || disallowed_robot_ids.size() == positions.size());

  std::vector<std::optional<Robot>> assignments(available_robots.size(), std::nullopt);

  if (available_robots.empty() || positions.empty()) {
    return assignments;
  }

  Eigen::MatrixXd costs = Eigen::MatrixXd::Constant(
    positions.size(),
    available_robots.size(), std::numeric_limits<double>::max());
  for (size_t i = 0; i < positions.size(); i++) {
    for (size_t j = 0; j < available_robots.size(); j++) {
      costs(i, j) = sqrt(CGAL::squared_distance(available_robots.at(j).pos, positions.at(i)));
      if(!disallowed_robot_ids.empty()) {
        const auto is_disallowed = std::ranges::find(disallowed_robot_ids[i], available_robots[j].id) != disallowed_robot_ids[i].end();
        if(is_disallowed) {
          costs(i,j) = std::numeric_limits<double>::max();
        }
      }
    }
  }

  const auto num_assignments = std::min(available_robots.size(), positions.size());

  for (auto row = 0ul; row < num_assignments; ++row) {
    Eigen::MatrixXd::Index min_index;
    costs.row(row).minCoeff(&min_index);
    assignments[row] = available_robots[min_index];
    costs.col(min_index).fill(std::numeric_limits<double>::max());
  }

  return assignments;
}

}  // namespace ateam_kenobi::play_helpers
