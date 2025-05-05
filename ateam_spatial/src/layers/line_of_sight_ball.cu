#include "ateam_spatial/layers/line_of_sight_ball.hpp"
#include "ateam_spatial/layers/line_of_sight.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float LineOfSightBall(const int x, const int y, const Ball & ball, const Robot * their_bots, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  const auto real_x = SpatialToRealX(x, field_dims, settings);
  const auto real_y = SpatialToRealY(y, field_dims, settings);

  return LineOfSight(real_x, real_y, ball.x, ball.y, their_bots, field_dims, settings);
}

} // namespace ateam_spatial::layers
