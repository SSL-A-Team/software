#include "ateam_spatial/maps/receiver_position_quality.hpp"
#include "ateam_spatial/layers/distance_down_field.hpp"
#include "ateam_spatial/layers/in_field.hpp"

namespace ateam_spatial::maps
{

CUDA_HOSTDEV float ReceiverPositionQuality(const int x, const int y, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  return layers::InField(x, y, field_dims, settings) * layers::DistanceDownField(x, field_dims, settings);
}
  
}  // namespace ateam_spatial::maps
