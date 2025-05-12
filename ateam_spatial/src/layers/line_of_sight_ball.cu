#include "ateam_spatial/layers/line_of_sight_ball.hpp"
#include "ateam_spatial/layers/line_of_sight.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float LineOfSightBall(const float x, const float y, const Ball & ball, const Robot * their_bots, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  const auto real_x = x;
  const auto real_y = y;

  return LineOfSight(real_x, real_y, ball.x, ball.y, their_bots, field_dims, settings);
}

} // namespace ateam_spatial::layers
