#include "ateam_spatial/layers/in_field.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float InField(const float x, const float y, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  const auto pos_x_thresh = field_dims.field_length / 2.0;
  const auto neg_x_thresh = -field_dims.field_length / 2.0;
  const auto pos_y_thresh = field_dims.field_width / 2.0;
  const auto neg_y_thresh = -field_dims.field_width / 2.0;

  return (x <= pos_x_thresh) && (x >= neg_x_thresh) && (y <= pos_y_thresh) && (y >= neg_y_thresh);
}
  
} // namespace ateam_spatial::layers
