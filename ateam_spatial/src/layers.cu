#include "ateam_spatial/layers.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

CUDA_HOSTDEV float DistanceDownField(const int x, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  return SpatialToRealX(x, field_dims, settings) + (WorldWidthReal(field_dims) / 2.0);
}
