#ifndef PATH_PLANNING__PATH_PLANNER_HPP_
#define PATH_PLANNING__PATH_PLANNER_HPP_

#include <ateam_geometry/types.hpp>

namespace ateam_kenobi::path_planning
{

class PathPlanner
{
public:

  std::vector<ateam_geometry::Point> getPath(const ateam_geometry::Point & start, const ateam_geometry::Point & goal);

private:


};

}  // namespace ateam_kenobi::path_planning;

#endif  // PATH_PLANNING__PATH_PLANNER_HPP_
