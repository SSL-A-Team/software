#include "ateam_spatial/maps/receiver_position_quality.hpp"
#include "ateam_spatial/layers/distance_down_field.hpp"

CUDA_HOSTDEV float ReceiverPositionQuality(const int x, const int y, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  return DistanceDownField(x, field_dims, settings);
}
