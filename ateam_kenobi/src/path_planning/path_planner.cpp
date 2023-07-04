#include "path_planner.hpp"

namespace ateam_kenobi::path_planning
{

  std::vector<ateam_geometry::Point> PathPlanner::getPath(const ateam_geometry::Point &start, const ateam_geometry::Point &goal)
  {
    return {start, goal};
  }

} // namespace ateam_kenobi::path_planning
