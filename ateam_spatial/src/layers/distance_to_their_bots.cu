#include "ateam_spatial/layers/width_of_shot_on_goal.hpp"
#include "ateam_spatial/layers/line_of_sight.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float DistanceToTheirBots(const int x, const int y, const Robot * their_robots, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  const auto real_x = SpatialToRealX(x, field_dims, settings);
  const auto real_y = SpatialToRealY(y, field_dims, settings);
  auto result = MAXFLOAT;
  for(auto i = 0; i < 16; ++i) {
    const auto & robot = their_robots[i];
    if(!robot.visible) {
      continue;
    }
    const auto distance = hypotf(robot.x - real_x, robot.y - real_y);
    result = min(result, distance);
  }
  return result;
}

} // namespace ateam_spatial::layers
