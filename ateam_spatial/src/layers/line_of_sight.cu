#include "ateam_spatial/layers/line_of_sight_ball.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float PointDistance(const float x_1, const float y_1, const float x_2, const float y_2) {
  return hypotf((x_2 - x_1), (y_2 - y_1));
}

CUDA_HOSTDEV float DistanceToSegment(const float line_1_x, const float line_1_y, const float line_2_x, const float line_2_y, const float point_x, const float point_y)
{
  const auto l2 = PointDistance(line_1_x, line_1_y, line_2_x, line_2_y);
  if (fabsf(l2) < 1e-8f) {
    return PointDistance(line_1_x, line_1_y, point_x, point_y);
  }
  auto t = (((point_x - line_1_x) * (line_2_x - line_1_x)) + ((point_y - line_1_y)*(line_2_y - line_1_y))) / (l2*l2);
  t = max(0.0f, min(1.0f, t));
  const auto projected_x = line_1_x + (t * (line_2_x - line_1_x));
  const auto projected_y = line_1_y + (t * (line_2_y - line_1_y));
  return PointDistance(point_x, point_y, projected_x, projected_y);
}

CUDA_HOSTDEV float LineOfSight(const float src_x, const float src_y, const float dst_x, const float dst_y, const Robot * their_bots, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  const auto robot_radius = 0.09f;

  bool result = 1;
  for(auto i = 0; i < 16; ++i) {
    const auto & robot = their_bots[i];
    if(!robot.visible) {
      continue;
    }
    const auto distance = DistanceToSegment(dst_x, dst_y, src_x, src_y, robot.x, robot.y);
    result &= distance > robot_radius;
  }

  return result;
}

} // namespace ateam_spatial::layers
