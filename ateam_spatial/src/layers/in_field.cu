#include "ateam_spatial/layers/in_field.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float InField(const int x, const int y, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  const auto pos_x_thresh = RealToSpatialX(field_dims.field_length / 2.0, field_dims, settings);
  const auto neg_x_thresh = RealToSpatialX(-field_dims.field_length / 2.0, field_dims, settings);
  const auto pos_y_thresh = RealToSpatialY(field_dims.field_width / 2.0, field_dims, settings);
  const auto neg_y_thresh = RealToSpatialY(-field_dims.field_width / 2.0, field_dims, settings);

  return (x <= pos_x_thresh) && (x >= neg_x_thresh) && (y <= pos_y_thresh) && (y >= neg_y_thresh);
}
  
} // namespace ateam_spatial::layers
