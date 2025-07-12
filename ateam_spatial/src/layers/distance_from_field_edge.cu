#include "ateam_spatial/layers/distance_from_field_edge.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float DistanceFromFieldEdge(const float x, const float y, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  const auto x_real = x;
  const auto y_real = y;
  const auto half_world_width = WorldWidthReal(field_dims) / 2.0f;
  const auto half_world_height = WorldHeightReal(field_dims) / 2.0f;
  const auto half_world_diff = half_world_width - half_world_height;

  if(fabs(x_real) < half_world_diff || (fabs(x_real) - half_world_diff) < fabs(y_real)) {
    return half_world_height - fabs(y_real);
  } else {
    return half_world_width - fabs(x_real);
  }
}
  
} // namespace ateam_spatial::layers
