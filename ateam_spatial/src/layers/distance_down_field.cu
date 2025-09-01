#include "ateam_spatial/layers/distance_down_field.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float DistanceDownField(const float x, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  return x + (WorldWidthReal(field_dims) / 2.0f);
}
  
} // namespace ateam_spatial::layers
